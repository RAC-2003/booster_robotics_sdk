#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fastdds/dds/log/Log.hpp>

#include <booster/idl/b1/Kick.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_publisher.hpp>

using namespace eprosima::fastdds::dds;
using booster::robot::ChannelFactory;
using booster::robot::ChannelPublisher;
using booster::robot::b1::B1LocoClient;
using booster::robot::b1::VisualKickVersion;

// ============================================================================
// Vision: tmpfs file reader (replaces the SDK's VisionClient RPC path)
// ============================================================================
// booster::robot::vision::VisionClient's StartVisionService()/GetDetectionObject()
// never completes its handshake against this robot's vision_rpc_client_bridge
// (see the extensive diagnosis earlier this session - the SDK's own request
// writer never becomes discoverable on the wire, for reasons that remain
// unresolved after exhausting black-box diagnosis of the SDK/bridge pairing).
//
// robocup_demo's own vision pipeline (vision_node.cpp) is the actual working,
// production perception stack on this robot: it publishes fully-formed,
// robot-frame detections (ball + person + more) on a ROS2 topic,
// /booster_soccer/detection. A separate, standalone process
// (vision_ros_bridge_pkg's vision_bridge_node) subscribes to that topic and
// writes the latest detections to /dev/shm/booster_vision_bridge.txt.
//
// That has to be a genuinely separate OS PROCESS, not just a separate
// translation unit/library linked into this binary: ROS2 Humble's
// rmw_fastrtps_cpp needs libfastrtps.so.2.6, while the Booster SDK
// (ChannelFactory/B1LocoClient below) needs libfastrtps.so.2.13 - two
// ABI-incompatible versions of the same library. Confirmed by crash: an
// rclcpp::Node created in the same process as the SDK's FastDDS threw
// std::bad_array_new_length from corrupted struct layout inside
// rmw_fastrtps_shared_cpp::create_participant. Keeping ROS2 in its own
// process means each process only ever loads one FastDDS version.
//
// Everything downstream (the state machine, IsBall/IsPerson, the kick logic)
// is unchanged - it still operates on a vector of SimpleDetection with the
// same tag_/conf_/position_ fields, just populated by parsing a text file
// instead of an RPC response or a ROS2 callback.
struct SimpleDetection {
    std::string tag_;
    float conf_ = 0.0f;
    std::vector<float> position_;
};

class VisionFileReader {
public:
    explicit VisionFileReader(std::string path) : path_(std::move(path)) {}

    // Reads and parses the file. Returns true only if the file exists,
    // parses cleanly, and was written within max_age_ms (guards against a
    // dead/stuck vision_bridge_node leaving stale data behind).
    bool Poll(std::vector<SimpleDetection>& objects, long long max_age_ms = 500) {
        objects.clear();

        std::ifstream in(path_);
        if (!in.is_open()) {
            return false;
        }

        long long ts_ms = 0;
        long long capture_ts_ms = 0;
        in >> ts_ms >> capture_ts_ms;
        std::string discard;
        std::getline(in, discard);  // consume rest of the timestamp line

        std::string line;
        while (std::getline(in, line)) {
            if (line.empty()) continue;

            const size_t p1 = line.find('|');
            const size_t p2 = (p1 == std::string::npos) ? std::string::npos : line.find('|', p1 + 1);
            if (p1 == std::string::npos || p2 == std::string::npos) continue;

            SimpleDetection d;
            d.tag_ = line.substr(0, p1);
            try {
                d.conf_ = std::stof(line.substr(p1 + 1, p2 - p1 - 1));
            } catch (const std::exception&) {
                continue;
            }

            std::stringstream pos_stream(line.substr(p2 + 1));
            std::string value;
            while (std::getline(pos_stream, value, ',')) {
                if (value.empty()) continue;
                try {
                    d.position_.push_back(std::stof(value));
                } catch (const std::exception&) {
                    // skip malformed field, keep the rest of the detection
                }
            }
            objects.push_back(std::move(d));
        }

        received_at_least_one_ = true;

        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // capture_ts_ms is the original camera frame's own timestamp,
        // forwarded all the way from vision_node - (now_ms - capture_ts_ms)
        // is the *true* end-to-end pipeline latency (camera -> inference ->
        // ROS2 publish -> bridge file -> this poll), not just the last leg.
        // 0 means the source message had no header stamp set.
        last_pipeline_latency_ms_ = (capture_ts_ms > 0) ? (now_ms - capture_ts_ms) : -1;

        return (now_ms - ts_ms) <= max_age_ms;
    }

    bool HasReceivedAtLeastOne() const { return received_at_least_one_; }

    // -1 if unavailable (no header stamp forwarded).
    long long LastPipelineLatencyMs() const { return last_pipeline_latency_ms_; }

private:
    std::string path_;
    bool received_at_least_one_ = false;
    long long last_pipeline_latency_ms_ = -1;
};

static constexpr const char* kVisionBridgeFilePath = "/dev/shm/booster_vision_bridge.txt";

// ============================================================================
// Diagnostic configuration
// ============================================================================
// This file is intentionally instrumented for diagnosis, not optimization.
// The goal is to reveal exactly:
//   - which SDK call blocks or fails,
//   - how long it takes,
//   - which attempt fails,
//   - which startup phase was active,
//   - and which state the robot was in when the failure happened.
static constexpr bool kDebugVerbose = true;

// Heartbeat frequency for the main loop. Every N iterations a compact status
// summary is printed so we can see whether the program is making forward
// progress or stalling in one region.
static constexpr int kHeartbeatEveryLoops = 10;

// Motion logging can get noisy. These thresholds suppress logs when the command
// arguments are effectively unchanged.
static constexpr float kMoveLogThreshold = 0.01f;
static constexpr float kHeadLogThreshold = 0.01f;

// ============================================================================
// Global runtime state
// ============================================================================
static std::atomic<bool> g_run{true};
static const auto g_program_start = std::chrono::steady_clock::now();

// We keep a few global diagnostic breadcrumbs so that on exit we can summarize
// the last successful step, last failed call, total retries, etc.
static std::string g_last_successful_phase = "<none>";
static std::string g_last_successful_rpc = "<none>";
static std::string g_last_failed_rpc = "<none>";
static int g_total_retry_attempts = 0;
static bool g_vision_started_successfully = false;
static bool g_walking_mode_succeeded = false;
static bool g_any_detection_poll_succeeded = false;
static bool g_reached_kick_state = false;

// ============================================================================
// Logging framework
// ============================================================================
// We use a tiny in-file logger instead of an external logging library to keep
// this file portable and easy to drop into your workspace.
//
// Each log line includes:
//   - elapsed milliseconds since program start,
//   - log level,
//   - optional context,
//   - the message.
//
// Example:
//   [1234 ms] [INFO ] [phase] PHASE 4 START: StartVisionService
enum class LogLevel {
    kDebug,
    kInfo,
    kWarn,
    kError
};

static const char* ToString(LogLevel level) {
    switch (level) {
        case LogLevel::kDebug: return "DEBUG";
        case LogLevel::kInfo:  return "INFO ";
        case LogLevel::kWarn:  return "WARN ";
        case LogLevel::kError: return "ERROR";
        default:               return "UNKWN";
    }
}

static bool ShouldLog(LogLevel level) {
    if (kDebugVerbose) {
        return true;
    }
    return level != LogLevel::kDebug;
}

static long long ElapsedMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - g_program_start).count();
}

static void LogMsg(LogLevel level,
                const std::string& message,
                const std::string& context = "") {
    if (!ShouldLog(level)) {
        return;
    }

    std::ostream& os = (level == LogLevel::kError || level == LogLevel::kWarn)
                           ? std::cerr
                           : std::cout;

    os << "[" << ElapsedMs() << " ms]"
       << " [" << ToString(level) << "]";

    if (!context.empty()) {
        os << " [" << context << "]";
    }

    os << " " << message << "\n";
}

// ============================================================================
// State machine definition
// ============================================================================
enum class State {
    kSearchBall,
    kApproachBall,
    kSearchPerson,
    kAlignBody,
    kKick
};

static const char* StateName(State s) {
    switch (s) {
        case State::kSearchBall:   return "SEARCH_BALL";
        case State::kApproachBall: return "APPROACH_BALL";
        case State::kSearchPerson: return "SEARCH_PERSON";
        case State::kAlignBody:    return "ALIGN_BODY";
        case State::kKick:         return "KICK";
        default:                   return "UNKNOWN_STATE";
    }
}

static void LogStateTransition(State from, State to, const std::string& reason) {
    std::ostringstream oss;
    oss << StateName(from) << " -> " << StateName(to)
        << " | reason=" << reason;
    LogMsg(LogLevel::kInfo, oss.str(), "state");
}

// ============================================================================
// Signal handling
// ============================================================================
static void sigint_handler(int) {
    LogMsg(LogLevel::kWarn, "SIGINT received; requesting clean shutdown.", "signal");
    g_run = false;
}

// ============================================================================
// Timing / behavior constants
// ============================================================================
static constexpr int kPhaseInitDelayMs = 2000;
static constexpr int kPhaseServiceDelayMs = 1500;
static constexpr int kPhaseModeDelayMs = 1000;
static constexpr int kStartupPollMs = 120;
static constexpr int kSearchPollMs = 120;
static constexpr int kActivePollMs = 60;
static constexpr int kRpcMaxAttempts = 4;
static constexpr int kRpcBackoffBaseMs = 250;
static constexpr int kPassiveValidationCycles = 5;
static constexpr int kPassiveValidationFailureLimit = 2;
static constexpr int kPassiveValidationRpcAttempts = 2;
static constexpr int kActivePollFailureLimit = 2;
static constexpr int kTargetLossLimit = 3;
static constexpr int kSweepStepsHalf = 20;
static constexpr int kKickFrames = 20;
static constexpr int kKickPublishIntervalMs = 33;
// The physical kick motion (wind-up/swing/follow-through) takes real time to
// execute once VisualKick(true) succeeds - observed as a leg "jerk" that got
// cut off when VisualKick(false) was called too soon after. Give it a real
// window to complete before disarming. robocup_demo's RLVisionKick tree node
// (subtree_striker_play.xml) uses min_msec_kick=1500/max_msec_kick=7000 for
// this same window; 4000ms sits comfortably inside that reference range.
static constexpr int kKickMotionWaitMs = 4000;
// robocup_demo's Kick::onRunning (brain_tree.cpp) also aborts if the ball's
// range grows too far past its minimum-seen value during the kick (ball got
// knocked/moved away) - we had the same check here, but dropped it after
// confirming it false-positives on this robot (see PublishKickFrames for the
// full reasoning): the kick's own stance-shift motion moves the camera in a
// way indistinguishable from "the ball moved" using position alone.
// BALL_LOST_THRESHOLD equivalent: abort if no fresh ball detection arrives
// for this long during the kick.
static constexpr int kKickBallLostThresholdMs = 1000;
static constexpr int kSearchBodyRotateMs = 300;

// Exponential smoothing weight applied to raw ball_x/ball_y readings during
// APPROACH_BALL (see the comment at its use site). Lower = smoother/more lag,
// higher = more responsive/more noise passed through.
static constexpr float kBallPositionSmoothingAlpha = 0.4f;

// Maximum rate (m/s) the tracked ball_x/ball_y estimate is allowed to move
// per second of elapsed time, regardless of what a single raw reading
// claims - bounds how much one extreme outlier can yank the controller's
// belief in one step, without ever fully ignoring new data (see the comment
// at the use site for why a hard accept/reject filter didn't work here).
// A logged run showed ball_x jump 0.58m in 616ms (~0.94 m/s implied) right
// before the robot ran over the ball; set below that so a jump of that size
// gets meaningfully damped rather than passed straight through.
static constexpr double kMaxBallTrackingSpeedMps = 0.5;

// Plausibility baseline: the ball should only ever get closer as the robot
// walks toward it, so a reading claiming it's suddenly much farther away
// than the best distance seen so far is more likely a bad detection than
// the ball actually retreating. Established from an average of several
// readings right when the ball is first found (see SEARCH_BALL) rather than
// a single sample, to avoid seeding it from one noisy outlier.
//
// This is deliberately NOT a hard reject: an earlier hard accept/reject
// filter (based on implied speed between consecutive readings) got stuck
// permanently rejecting everything once its reference went stale, and lost
// the ball entirely. A fixed baseline avoids that specific staleness-drift
// failure mode, but introduces a different one - if the ball is genuinely
// bumped farther away (search-recovery, being kicked, knocked aside), a
// pure "never accept farther than baseline" rule would reject every
// reading forever after that with no way to recover. The escape hatch below
// exists specifically for that: a "too far" reading has to persist for
// several consecutive polls (i.e. it's consistent, not a one-off outlier)
// before it's accepted as the new reality and the baseline moves out to it.
static constexpr int kBaselineSampleCount = 10;
static constexpr int kBaselineSampleIntervalMs = 30;
static constexpr float kPlausibilityMarginM = 0.15f;
static constexpr int kPlausibilityEscapeHatchPolls = 5;

// Was 0.22-0.45m, then widened to 0.40-0.65m - still not enough: the robot
// kept running over the ball even with a firm post-detection stop
// (HardStopAndSettle), meaning the overrun happens *before* "ready range" is
// even detected, not from momentum after. Root cause is almost certainly
// vision pipeline latency (camera -> inference -> bridge file -> poll): the
// approach speed acts on a ball_x reading that's already stale by the time
// it's used, so the robot has physically closed more distance than the
// controller's input reflects. Widened further for margin, paired with a
// slower max approach speed below to shrink the lag-distance itself.
// Increased after a run where the robot physically made contact with the
// ball despite stopping within the old 0.55-0.85 window (measured ball_x was
// 0.598m at the stop). ball_x is reported relative to the robot's base/pelvis
// frame, not its feet - a leading foot mid-stride can reach well forward of
// that origin, plausibly 20-30cm, so a pelvis-frame reading of "0.6m away"
// doesn't guarantee that much real clearance to the ball. Widened to build in
// more margin for that stride reach.
//
// That widened range (0.75-1.05) then turned out too far - the robot
// stopped at 1.047m and the kick's leg swing didn't reach the ball at all.
// A 0.3m-wide window also means the actual stop point varies a lot between
// runs depending on exactly when a reading crosses into it, making it hard
// to pin down the right distance empirically. Narrowed to a tight band at
// the user-requested ~0.4-0.5m target instead, so each test actually
// measures that specific distance rather than "somewhere in a wide range
// again."
static constexpr float kBallReadyDistMin = 0.4f;
static constexpr float kBallReadyDistMax = 0.5f;
// Minimum forward speed while still meaningfully far from the target,
// matching robocup_demo's RobotClient::setVelocity min_vx floor pattern -
// below this, the robot doesn't take clean steps, just rocks in place. See
// the comment at the vx computation for the full story.
static constexpr double kApproachVxMin = 0.15;
// Only enforce that floor while ex is above this - close to the target
// (small ex), let vx decay naturally below the floor instead. The ready-
// range check requires both ex in range AND tight lateral (ey) alignment;
// if the robot gets close but is still correcting sideways, flooring vx at
// a real walking speed meant walking right through the ball while waiting
// for lateral alignment to catch up, instead of finishing the approach slowly.
static constexpr double kApproachVxMinZoneThreshold = 0.3;
static constexpr float kBallAlignY = 0.05f;
static constexpr float kAlignYawThresh = 0.35f;
static constexpr float kMinPersonFwdDist = 0.10f;
// Was 0.30-0.90 - per HTWK Robots (RoboCup team with a working kick_ball
// implementation on this same platform), their strength values run 2.3
// (normal) to 8.0 (hard kick), a totally different scale. A kick published
// at ~10% of the intended strength would plausibly look like "aimed fine but
// doesn't connect/barely moves the ball" - exactly what we were seeing.
static constexpr float kPowerMin = 2.0f;
static constexpr float kPowerMax = 8.0f;
static constexpr float kPowerDist = 4.0f;
static constexpr float kBallSearchBodyYawRate = 0.25f;
static constexpr float kPersonSearchBodyYawRate = 0.30f;
static constexpr float kAlignBodyYawRate = 0.30f;

// ============================================================================
// Formatting helpers
// ============================================================================
// These helpers make log output readable and safe even when a vector is empty
// or malformed.
static std::string FormatXY(float x, float y) {
    std::ostringstream oss;
    oss << "(" << std::fixed << std::setprecision(3) << x << ", " << y << ")";
    return oss.str();
}

static std::string FormatVecXY(const std::vector<float>& v) {
    if (v.empty()) {
        return "<empty>";
    }
    if (v.size() < 2) {
        return "<invalid:size=" + std::to_string(v.size()) + ">";
    }
    return FormatXY(v[0], v[1]);
}

static std::string FormatBool(bool value) {
    return value ? "true" : "false";
}

// ============================================================================
// Tag / geometry helpers
// ============================================================================
static bool IsBall(const std::string& tag) {
    return tag == "ball" || tag == "Ball";
}

static bool IsPerson(const std::string& tag) {
    return tag == "face" || tag == "Face" ||
           tag == "person" || tag == "Person";
}

static bool HasRobotFrameXY(const SimpleDetection& obj) {
    return obj.position_.size() >= 2;
}

static float Dist2D(const std::vector<float>& pos) {
    if (pos.size() < 2) {
        return 9999.0f;
    }
    return std::hypot(pos[0], pos[1]);
}

static const SimpleDetection* FindBestBall(const std::vector<SimpleDetection>& objects) {
    const SimpleDetection* best_ball = nullptr;
    for (const auto& object : objects) {
        if (IsBall(object.tag_) && HasRobotFrameXY(object)) {
            if (!best_ball || object.conf_ > best_ball->conf_) {
                best_ball = &object;
            }
        }
    }
    return best_ball;
}

static void SleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static int PollDelayMsForState(State state) {
    return (state == State::kApproachBall || state == State::kAlignBody)
               ? kActivePollMs
               : kSearchPollMs;
}

// ============================================================================
// Scoped timer for phase / scope duration logs
// ============================================================================
// This is useful for phase markers such as:
//   PHASE 4 START
//   PHASE 4 END duration_ms=...
class ScopedPhaseTimer {
public:
    ScopedPhaseTimer(std::string name, std::string context = "phase")
        : name_(std::move(name)),
          context_(std::move(context)),
          start_(std::chrono::steady_clock::now()) {
        LogMsg(LogLevel::kInfo, name_ + " START", context_);
    }

    ~ScopedPhaseTimer() {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_).count();
        LogMsg(LogLevel::kInfo,
            name_ + " END duration_ms=" + std::to_string(elapsed),
            context_);
    }

private:
    std::string name_;
    std::string context_;
    std::chrono::steady_clock::time_point start_;
};

// ============================================================================
// Retry wrapper for SDK calls returning int
// ============================================================================
// This helper logs:
//   - ENTER of each attempt
//   - per-attempt duration
//   - rc value
//   - retry sleep
//   - final success/failure
//
// It is one of the most important diagnostic pieces, because if the timeout is
// thrown from an SDK call we want to know:
//   - which call,
//   - which attempt,
//   - and how long it spent before returning.
static int RetryRpc(const std::string& name,
                    const std::function<int(void)>& fn,
                    int max_attempts = kRpcMaxAttempts) {
    const auto total_start = std::chrono::steady_clock::now();
    int rc = -1;

    LogMsg(LogLevel::kDebug,
        "ENTER retry wrapper for " + name + " max_attempts=" + std::to_string(max_attempts),
        "rpc");

    for (int attempt = 1; attempt <= max_attempts && g_run.load(); ++attempt) {
        ++g_total_retry_attempts;

        const auto attempt_start = std::chrono::steady_clock::now();
        LogMsg(LogLevel::kInfo,
            "ENTER " + name + " attempt=" + std::to_string(attempt) + "/" +
                std::to_string(max_attempts),
            "rpc");

        rc = fn();

        const auto attempt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - attempt_start).count();

        LogMsg(LogLevel::kInfo,
            "EXIT " + name + " attempt=" + std::to_string(attempt) +
                " rc=" + std::to_string(rc) +
                " duration_ms=" + std::to_string(attempt_ms),
            "rpc");

        if (rc == 0) {
            g_last_successful_rpc = name;

            const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - total_start).count();

            LogMsg(LogLevel::kInfo,
                name + " SUCCESS total_duration_ms=" + std::to_string(total_ms) +
                    " attempts_used=" + std::to_string(attempt),
                "rpc");
            return 0;
        }

        g_last_failed_rpc = name;

        if (attempt < max_attempts) {
            const int backoff_ms = kRpcBackoffBaseMs * attempt;
            LogMsg(LogLevel::kWarn,
                name + " failed rc=" + std::to_string(rc) +
                    " attempt=" + std::to_string(attempt) +
                    " sleeping_backoff_ms=" + std::to_string(backoff_ms),
                "rpc");
            SleepMs(backoff_ms);
        }
    }

    const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - total_start).count();

    LogMsg(LogLevel::kError,
        name + " FINAL FAILURE rc=" + std::to_string(rc) +
            " total_duration_ms=" + std::to_string(total_ms),
        "rpc");
    return rc;
}

// ============================================================================
// Vision polling (tmpfs file read, replaces the RPC retry wrapper)
// ============================================================================
// There is no RPC round-trip anymore, so there is no timeout/retry to manage -
// this just re-reads the file vision_bridge_node keeps overwriting. Note: the
// SDK path's per-request "ratio" (focus-region narrowing during
// APPROACH_BALL) has no equivalent here, since vision_node.cpp doesn't take a
// per-request parameter over the topic - it always publishes full-frame
// detections. That narrowing is dropped rather than reimplemented client-side.
static bool PollDetections(VisionFileReader& vision_reader,
                           std::vector<SimpleDetection>& objects) {
    const bool ok = vision_reader.Poll(objects);

    if (!objects.empty() || vision_reader.HasReceivedAtLeastOne()) {
        g_any_detection_poll_succeeded = true;
    }

    return ok;
}

// ============================================================================
// Self-motion dead reckoning (perception-latency compensation)
// ============================================================================
// Measured end-to-end pipeline_latency_ms (see VisionFileReader) runs ~2.2s
// on this robot - the bottleneck turned out to be the vendor's own camera
// streaming process (cam_stream_receiver_ros2) running stereo depth
// inference we never consume, not anything in vision_node or this file. A
// ball reading is therefore already ~2.2s stale by the time we act on it: at
// the approach cruise speed (~0.2 m/s) that is up to ~0.44m of blind travel
// per reading - enough on its own to explain "the logs show it stopping at
// the right distance, but it physically ran through the ball", since the
// logged distance was correct for 2.2 seconds ago, not for the moment the
// stop decision actually took effect.
//
// There is no real odometry wired up here (deferred). As a proxy, every
// commanded velocity is recorded (from LoggedMove, the single choke point
// all motion commands pass through) with a timestamp, and we dead-reckon the
// robot's net displacement since a given reading's capture time by
// integrating that command history forward with simple unicycle kinematics.
// This assumes the robot tracks its commanded velocity reasonably closely -
// not exact, but far closer to reality than treating a 2.2s-old reading as
// "now" outright, which is what the approach controller did before this.
struct VelocityCommand {
    std::chrono::steady_clock::time_point time;
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
};

class VelocityHistory {
public:
    void Record(double vx, double vy, double wz) {
        const auto now = std::chrono::steady_clock::now();
        history_.push_back({now, vx, vy, wz});
        while (!history_.empty() &&
               std::chrono::duration<double>(now - history_.front().time).count() > kMaxHistorySeconds) {
            history_.pop_front();
        }
    }

    // Integrates recorded commands from `since` to now, returning the net
    // displacement (dx, dy) expressed in the body-frame axes as they were
    // oriented at `since`, plus the net heading change dtheta (positive =
    // turned left/CCW, matching this program's wz/yaw sign convention).
    void EstimateDisplacementSince(std::chrono::steady_clock::time_point since,
                                    double& out_dx, double& out_dy, double& out_dtheta) const {
        double x = 0.0, y = 0.0, theta = 0.0;
        auto t_cursor = since;
        double active_vx = 0.0, active_vy = 0.0, active_wz = 0.0;

        // The velocity active at `since` is whatever was last commanded at
        // or before it.
        for (const auto& cmd : history_) {
            if (cmd.time <= since) {
                active_vx = cmd.vx;
                active_vy = cmd.vy;
                active_wz = cmd.wz;
            } else {
                break;
            }
        }

        auto integrate_to = [&](std::chrono::steady_clock::time_point t_to) {
            const double dt = std::chrono::duration<double>(t_to - t_cursor).count();
            if (dt > 0.0) {
                x += (active_vx * std::cos(theta) - active_vy * std::sin(theta)) * dt;
                y += (active_vx * std::sin(theta) + active_vy * std::cos(theta)) * dt;
                theta += active_wz * dt;
            }
            t_cursor = t_to;
        };

        for (const auto& cmd : history_) {
            if (cmd.time <= since) continue;
            integrate_to(cmd.time);
            active_vx = cmd.vx;
            active_vy = cmd.vy;
            active_wz = cmd.wz;
        }
        integrate_to(std::chrono::steady_clock::now());

        out_dx = x;
        out_dy = y;
        out_dtheta = theta;
    }

private:
    static constexpr double kMaxHistorySeconds = 4.0;
    std::deque<VelocityCommand> history_;
};

static VelocityHistory g_velocity_history;

// Projects a ball reading taken latency_ms ago forward to "now" by removing
// the robot's own estimated motion over that window. No-op if latency data
// isn't available (<= 0, i.e. -1 sentinel from VisionFileReader).
static void CompensateForLatency(float& ball_x, float& ball_y, long long latency_ms) {
    if (latency_ms <= 0) return;

    const auto capture_time =
        std::chrono::steady_clock::now() - std::chrono::milliseconds(latency_ms);

    double dx = 0.0, dy = 0.0, dtheta = 0.0;
    g_velocity_history.EstimateDisplacementSince(capture_time, dx, dy, dtheta);

    // Ball position relative to the robot's new position, still expressed in
    // the frame-at-capture-time axes.
    const double rel_x = static_cast<double>(ball_x) - dx;
    const double rel_y = static_cast<double>(ball_y) - dy;

    // Rotate into the robot's *current* body frame (it has turned by dtheta
    // since capture).
    const double cos_t = std::cos(dtheta);
    const double sin_t = std::sin(dtheta);
    const float compensated_x = static_cast<float>(rel_x * cos_t + rel_y * sin_t);
    const float compensated_y = static_cast<float>(-rel_x * sin_t + rel_y * cos_t);

    LogMsg(LogLevel::kDebug,
        "Latency compensation: raw=(" + std::to_string(ball_x) + ", " + std::to_string(ball_y) +
            ") latency_ms=" + std::to_string(latency_ms) +
            " self_motion dx=" + std::to_string(dx) + " dy=" + std::to_string(dy) +
            " dtheta=" + std::to_string(dtheta) +
            " -> compensated=(" + std::to_string(compensated_x) + ", " + std::to_string(compensated_y) + ")",
        "vision");

    ball_x = compensated_x;
    ball_y = compensated_y;
}

// ============================================================================
// Motion wrappers with compact logging
// ============================================================================
// We do not want to flood logs with identical Move/RotateHead commands every
// loop, so these wrappers only log when the command changes meaningfully.
struct MotionLogState {
    bool move_valid = false;
    float move_x = 0.0f;
    float move_y = 0.0f;
    float move_yaw = 0.0f;

    bool head_valid = false;
    float head_pitch = 0.0f;
    float head_yaw = 0.0f;

    bool head_sweep_valid = false;
    int head_sweep_pitch_dir = 0;
    int head_sweep_yaw_dir = 0;
};

static MotionLogState g_motion_log_state;

static bool NearlyDifferent(float a, float b, float threshold) {
    return std::abs(a - b) > threshold;
}

static int LoggedMove(B1LocoClient& loco, float x, float y, float yaw, const std::string& why) {
    g_velocity_history.Record(x, y, yaw);

    const bool should_log =
        !g_motion_log_state.move_valid ||
        NearlyDifferent(g_motion_log_state.move_x, x, kMoveLogThreshold) ||
        NearlyDifferent(g_motion_log_state.move_y, y, kMoveLogThreshold) ||
        NearlyDifferent(g_motion_log_state.move_yaw, yaw, kMoveLogThreshold);

    if (should_log) {
        LogMsg(LogLevel::kDebug,
            "ENTER loco.Move x=" + std::to_string(x) +
                " y=" + std::to_string(y) +
                " yaw=" + std::to_string(yaw) +
                " why=" + why,
            "motion");
    }

    const auto start = std::chrono::steady_clock::now();
    int rc = loco.Move(x, y, yaw);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    if (should_log) {
        LogMsg(LogLevel::kDebug,
            "EXIT loco.Move rc=" + std::to_string(rc) +
                " duration_ms=" + std::to_string(ms),
            "motion");

        g_motion_log_state.move_valid = true;
        g_motion_log_state.move_x = x;
        g_motion_log_state.move_y = y;
        g_motion_log_state.move_yaw = yaw;
    }

    return rc;
}

static int LoggedRotateHead(B1LocoClient& loco, float pitch, float yaw, const std::string& why) {
    const bool should_log =
        !g_motion_log_state.head_valid ||
        NearlyDifferent(g_motion_log_state.head_pitch, pitch, kHeadLogThreshold) ||
        NearlyDifferent(g_motion_log_state.head_yaw, yaw, kHeadLogThreshold);

    if (should_log) {
        LogMsg(LogLevel::kDebug,
            "ENTER loco.RotateHead pitch=" + std::to_string(pitch) +
                " yaw=" + std::to_string(yaw) +
                " why=" + why,
            "motion");
    }

    const auto start = std::chrono::steady_clock::now();
    int rc = loco.RotateHead(pitch, yaw);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    if (should_log) {
        LogMsg(LogLevel::kDebug,
            "EXIT loco.RotateHead rc=" + std::to_string(rc) +
                " duration_ms=" + std::to_string(ms),
            "motion");

        g_motion_log_state.head_valid = true;
        g_motion_log_state.head_pitch = pitch;
        g_motion_log_state.head_yaw = yaw;
    }

    return rc;
}

static int LoggedRotateHeadWithDirection(B1LocoClient& loco,
                                         int pitch_direction,
                                         int yaw_direction,
                                         const std::string& why) {
    const bool should_log =
        !g_motion_log_state.head_sweep_valid ||
        g_motion_log_state.head_sweep_pitch_dir != pitch_direction ||
        g_motion_log_state.head_sweep_yaw_dir != yaw_direction;

    if (should_log) {
        LogMsg(LogLevel::kDebug,
            "ENTER loco.RotateHeadWithDirection pitch_direction=" +
                std::to_string(pitch_direction) +
                " yaw_direction=" + std::to_string(yaw_direction) +
                " why=" + why,
            "motion");
    }

    const auto start = std::chrono::steady_clock::now();
    int rc = loco.RotateHeadWithDirection(pitch_direction, yaw_direction);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    if (should_log) {
        LogMsg(LogLevel::kDebug,
            "EXIT loco.RotateHeadWithDirection rc=" + std::to_string(rc) +
                " duration_ms=" + std::to_string(ms),
            "motion");

        g_motion_log_state.head_sweep_valid = true;
        g_motion_log_state.head_sweep_pitch_dir = pitch_direction;
        g_motion_log_state.head_sweep_yaw_dir = yaw_direction;
    }

    return rc;
}

// ============================================================================
// Safety / recovery helpers
// ============================================================================
static void StopMotionAndCenterHead(B1LocoClient& loco) {
    LogMsg(LogLevel::kInfo, "StopMotionAndCenterHead invoked", "safe");
    (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "stop motion");
    (void)LoggedRotateHead(loco, 0.0f, 0.0f, "center head");
}

// Stops body motion only, deliberately leaving the head wherever it
// currently is. Used right after SEARCH_BALL/SEARCH_PERSON find their
// target: unconditionally centering the head here (the old behavior, via
// StopMotionAndCenterHead) could swing the camera's view away from a target
// that was only in frame at an off-center search angle in the first place,
// immediately losing it again and bouncing back into search - observed as
// the robot repeatedly stopping, "looking" at the ball, then losing it and
// re-searching without ever making forward progress.
static void StopMotion(B1LocoClient& loco) {
    LogMsg(LogLevel::kInfo, "StopMotion invoked (head left in place)", "safe");
    (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "stop motion");
}

// A single Move(0,0,0) sets the target velocity to zero but doesn't
// instantly cancel a walking gait's momentum - observed as the robot
// physically stepping/sliding over the ball right after "stopping" at the
// ready-range boundary. Hold a zero-velocity command for a real settle
// window so residual motion actually bleeds off before anything else (head
// search, body rotation) happens near the ball.
static constexpr int kHardStopSettleMs = 800;
static constexpr int kHardStopIntervalMs = 100;

static void HardStopAndSettle(B1LocoClient& loco) {
    LogMsg(LogLevel::kInfo,
        "HardStopAndSettle invoked (holding zero velocity for " +
            std::to_string(kHardStopSettleMs) + "ms to let momentum bleed off)",
        "safe");
    for (int elapsed = 0; elapsed < kHardStopSettleMs; elapsed += kHardStopIntervalMs) {
        (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "hard stop settle");
        SleepMs(kHardStopIntervalMs);
    }
    (void)LoggedRotateHead(loco, 0.0f, 0.0f, "center head");
}

static void EnterSafeState(B1LocoClient& loco) {
    LogMsg(LogLevel::kWarn,
        "Entering safe state: stop motion, center head, best-effort walking mode.",
        "safe");
    StopMotionAndCenterHead(loco);

    (void)RetryRpc("loco.ChangeMode(kWalking)", [&]() -> int {
        return loco.ChangeMode(booster::robot::RobotMode::kWalking);
    }, 2);
}

static void RecoverToSearchBall(B1LocoClient& loco,
                                State& state,
                                int& sweep_dir,
                                int& sweep_count,
                                int& ball_lost_count,
                                int& person_lost_count,
                                std::vector<float>& person_pos,
                                const std::string& reason) {
    LogMsg(LogLevel::kWarn, "RecoverToSearchBall reason=" + reason, "recover");

    const State old_state = state;

    StopMotionAndCenterHead(loco);
    person_pos.clear();
    sweep_dir = 1;
    sweep_count = 0;
    ball_lost_count = 0;
    person_lost_count = 0;
    state = State::kSearchBall;

    LogStateTransition(old_state, state, reason);
}

// ============================================================================
// Wait for the vision topic to actually be alive before entering the state
// machine (replaces the RPC-based PassiveVisionValidation).
// ============================================================================
static bool WaitForVisionTopic(VisionFileReader& vision_reader) {
    ScopedPhaseTimer timer("PHASE 5: wait for vision bridge file");

    LogMsg(LogLevel::kInfo,
        "Polling " + std::string(kVisionBridgeFilePath) + " without robot motion, "
        "to confirm vision_bridge_node (and robocup_demo's vision_node behind "
        "it) is actually running before entering the main state machine.",
        "vision");

    for (int cycle = 0; cycle < kPassiveValidationCycles && g_run.load(); ++cycle) {
        std::vector<SimpleDetection> objects;
        (void)PollDetections(vision_reader, objects);

        LogMsg(LogLevel::kInfo,
            "Wait-for-vision-file cycle=" + std::to_string(cycle + 1) + "/" +
                std::to_string(kPassiveValidationCycles) +
                " received_any=" + FormatBool(vision_reader.HasReceivedAtLeastOne()) +
                " object_count=" + std::to_string(objects.size()),
            "vision");

        if (vision_reader.HasReceivedAtLeastOne()) {
            return true;
        }

        SleepMs(kStartupPollMs);
    }

    LogMsg(LogLevel::kError,
        "No data ever appeared at " + std::string(kVisionBridgeFilePath) + ". Is "
        "vision_bridge_node running (a separate process - see "
        "vision_ros_bridge_pkg/src/vision_bridge_node.cpp), and is "
        "robocup_demo's vision_node/soccer agent/camera pipeline behind it "
        "actually running?",
        "vision");
    return false;
}

// ============================================================================
// Kick reference publisher
//
// Published via the SDK's own ChannelPublisher<MSG> (booster/robot/channel/
// channel_publisher.hpp), which creates its channel through the same
// ChannelFactory already Init'd in PHASE 1 for RPC traffic - matching the
// pattern in the SDK's own example/low_level/low_level_publisher.cpp - rather
// than a separate hand-rolled raw FastDDS participant. The raw-participant
// approach never produced a single successful write post-firmware-1.6
// (RETCODE_ERROR on every frame, even once matched with a subscriber), and
// test.cpp - the last known-working reference - failed identically, so this
// isn't a bug in either program: the raw-DDS approach itself doesn't work
// against the new firmware. Going through the SDK's own channel machinery is
// the officially supported path (per booster_robotics_sdk_latest's firmware
// 1.6 update). VisualKick(true/false, VisualKickVersion) and the power
// calculation are unchanged - only this transport layer is replaced.
// ============================================================================
// robocup_demo's RLVisionKick::onRunning (brain_tree.cpp) tilts the head
// down in two steps while the kick executes, so the camera can actually see
// the ball at close range instead of it being below the level-head field of
// view: pitch 0.4 rad for the first 300ms, then 0.7 rad from 300-550ms, then
// left there (no further head commands after 550ms). Adopting just this
// angle schedule - not the rest of RLVisionKick's crab-walk/deceleration
// machinery - since it's what let the ball stay visible long enough for our
// new ball-moved/ball-lost checks above to actually have live data to check
// against, instead of losing the ball to FOV almost immediately.
static constexpr float kKickHeadPitchStage1 = 0.4f;
static constexpr float kKickHeadPitchStage2 = 0.7f;
static constexpr int kKickHeadPitchStage1Ms = 300;
static constexpr int kKickHeadPitchStage2Ms = 550;

static bool PublishKickFrames(B1LocoClient& loco,
                              VisionFileReader& vision_reader,
                              float ball_x,
                              float ball_y,
                              const std::vector<float>& person_pos,
                              float power) {
    LogMsg(LogLevel::kInfo,
        "PublishKickFrames begin ball=" + FormatXY(ball_x, ball_y) +
            " person=" + FormatVecXY(person_pos) +
            " power=" + std::to_string(power),
        "kick");

    ChannelPublisher<brain::msg::Kick> publisher(booster::robot::b1::kTopicKickReference);
    publisher.InitChannel();

    // A freshly-created channel needs a brief moment for DDS discovery to
    // match it with the robot's kick-reference subscriber before writes
    // reliably land.
    {
        const auto wait_start = std::chrono::steady_clock::now();
        size_t match_count = publisher.GetMatchedSubscriptionsCount();
        while (match_count == 0 && g_run.load()) {
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - wait_start).count();
            if (elapsed_ms > 5000) {
                LogMsg(LogLevel::kWarn,
                    "No kick_ball subscriber matched after 5000ms of waiting.",
                    "kick");
                break;
            }
            SleepMs(100);
            match_count = publisher.GetMatchedSubscriptionsCount();
        }
        if (match_count > 0) {
            const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - wait_start).count();
            LogMsg(LogLevel::kInfo,
                "kick_ball channel matched after " + std::to_string(total_ms) + "ms.",
                "kick");
        }
    }

    // Message field semantics, per HTWK Robots (RoboCup team with a
    // confirmed-working kick_ball implementation on this same platform,
    // shared directly): dir is unused by the controller - always send 0.0.
    // goal_x/goal_y (robot-relative, same frame as ball x/y) is what
    // actually steers the kick: "where do you want the ball to go". We
    // previously computed dir as the ball-to-target bearing based on
    // robocup_demo's brain_tree.cpp (CalcKickDir/pubKickMsg), which does
    // set and use a dir field - but that's evidently not what this
    // platform's VisualKick/kick_ball controller reads. goal_x/goal_y are
    // left as person_pos (fixed for the whole kick, unlike ball_x/ball_y
    // below), matching "kick it toward where the person currently is."
    // header.stamp: brain stamps every message with the current time; set
    // here for parity even though its effect on the receiving controller is
    // unconfirmed.
    //
    // robocup_demo's Kick::onRunning (brain_tree.cpp) doesn't publish once
    // and go silent - it re-ticks every cycle for the whole kick execution
    // window. robocup_demo's version aborts if the ball's live range grows
    // too far past its minimum-seen value (ball got knocked away). We had
    // the same check here, but confirmed it unusable on this robot: once
    // VisualKick(true) engages, the robot's internal kick controller shifts
    // into a kicking stance (weight/leg motion) before the actual swing,
    // moving the camera in a way we have no visibility into or ability to
    // compensate for (our own commanded-velocity history shows zero motion
    // during this window, yet readings shift right along with the physical
    // stance change). Confirmed false-positive directly: aborted on a
    // provably stationary, untouched ball. Every abort cuts the kick off
    // mid-wind-up, which is why it looked like "aligned then backed away
    // instead of kicking" rather than a completed kick. Keeping only the
    // ball-lost-entirely check below, which doesn't depend on position
    // accuracy - just whether a Ball is detected at all.
    float min_range = Dist2D({ball_x, ball_y});
    auto last_ball_seen = std::chrono::steady_clock::now();

    int succeeded = 0;
    int total_frames = 0;
    const auto kick_start = std::chrono::steady_clock::now();
    bool aborted = false;

    while (g_run.load()) {
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - kick_start).count();
        if (elapsed_ms >= kKickMotionWaitMs) {
            break;
        }

        if (elapsed_ms < kKickHeadPitchStage1Ms) {
            (void)LoggedRotateHead(loco, kKickHeadPitchStage1, 0.0f, "look down at ball during kick (stage 1)");
        } else if (elapsed_ms < kKickHeadPitchStage2Ms) {
            (void)LoggedRotateHead(loco, kKickHeadPitchStage2, 0.0f, "look down at ball during kick (stage 2)");
        }

        std::vector<SimpleDetection> objects;
        (void)PollDetections(vision_reader, objects);

        const SimpleDetection* fresh_ball = nullptr;
        for (const auto& object : objects) {
            if (IsBall(object.tag_) && HasRobotFrameXY(object)) {
                if (!fresh_ball || object.conf_ > fresh_ball->conf_) {
                    fresh_ball = &object;
                }
            }
        }

        if (fresh_ball) {
            ball_x = fresh_ball->position_[0];
            ball_y = fresh_ball->position_[1];
            last_ball_seen = std::chrono::steady_clock::now();

            const float current_range = Dist2D({ball_x, ball_y});
            if (current_range < min_range) {
                min_range = current_range;
            }
            LogMsg(LogLevel::kDebug,
                "Kick poll ball=" + FormatXY(ball_x, ball_y) +
                    " range=" + std::to_string(current_range) +
                    " min_range=" + std::to_string(min_range) +
                    " elapsed_ms=" + std::to_string(elapsed_ms),
                "kick");
        } else {
            const auto since_seen_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_ball_seen).count();
            if (since_seen_ms > kKickBallLostThresholdMs) {
                LogMsg(LogLevel::kWarn,
                    "Ball lost during kick (no detection for " + std::to_string(since_seen_ms) +
                        "ms), aborting.",
                    "kick");
                aborted = true;
                break;
            }
        }

        const auto now = std::chrono::system_clock::now().time_since_epoch();
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

        std_msgs::msg::Header header;
        header.stamp().sec(static_cast<int32_t>(now_ns / 1000000000LL));
        header.stamp().nanosec(static_cast<uint32_t>(now_ns % 1000000000LL));

        brain::msg::Kick msg;
        msg.header(header);
        msg.x(ball_x);
        msg.y(ball_y);
        // Per HTWK Robots (working kick_ball implementation on this same
        // platform): dir is unused by the controller - goal_x/goal_y is
        // what actually steers the kick. Always 0, matching their reference.
        msg.dir(0.0);
        msg.goal_x(person_pos[0]);
        msg.goal_y(person_pos[1]);
        msg.power(power);

        ++total_frames;
        if (publisher.Write(&msg)) {
            ++succeeded;
        } else {
            LogMsg(LogLevel::kWarn,
                "Failed to publish kick frame " + std::to_string(total_frames), "kick");
        }

        SleepMs(kKickPublishIntervalMs);
    }

    publisher.CloseChannel();

    if (succeeded == 0) {
        LogMsg(LogLevel::kError, "PublishKickFrames: all frames failed to publish.", "kick");
        return false;
    }

    LogMsg(LogLevel::kInfo,
        "PublishKickFrames completed: " + std::to_string(succeeded) + "/" +
            std::to_string(total_frames) + " frames published successfully" +
            (aborted ? " (aborted early)." : "."),
        "kick");
    return true;
}

// ============================================================================
// Failure summary
// ============================================================================
static void PrintFailureSummary(State final_state) {
    std::ostringstream oss;
    oss << "\n========== DIAGNOSTIC SUMMARY ==========\n"
        << "total_runtime_ms          : " << ElapsedMs() << "\n"
        << "final_state               : " << StateName(final_state) << "\n"
        << "last_successful_phase     : " << g_last_successful_phase << "\n"
        << "last_successful_rpc       : " << g_last_successful_rpc << "\n"
        << "last_failed_rpc           : " << g_last_failed_rpc << "\n"
        << "total_retry_attempts      : " << g_total_retry_attempts << "\n"
        << "vision_started_successfully: " << FormatBool(g_vision_started_successfully) << "\n"
        << "walking_mode_succeeded    : " << FormatBool(g_walking_mode_succeeded) << "\n"
        << "any_detection_poll_succeeded: " << FormatBool(g_any_detection_poll_succeeded) << "\n"
        << "reached_kick_state        : " << FormatBool(g_reached_kick_state) << "\n"
        << "========================================";

    LogMsg(LogLevel::kInfo, oss.str(), "summary");
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <networkInterface>\n"
                  << "Example: " << argv[0] << " eth0\n";
        return 1;
    }

    std::signal(SIGINT, sigint_handler);

    Log::SetVerbosity(Log::Kind::Info);
    Log::ReportFunctions(true);

    const std::string network_interface = argv[1];
    LogMsg(LogLevel::kInfo,
        "Program start. network_interface=" + network_interface,
        "main");

    // ------------------------------------------------------------------------
    // PHASE 1: ChannelFactory::Init
    // ------------------------------------------------------------------------
    {
        ScopedPhaseTimer phase("PHASE 1: ChannelFactory::Init");

        LogMsg(LogLevel::kInfo,
            "ENTER ChannelFactory::Instance()->Init domain=0 interface=" + network_interface,
            "init");
        const auto start = std::chrono::steady_clock::now();
        ChannelFactory::Instance()->Init(0, network_interface);
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        LogMsg(LogLevel::kInfo,
            "EXIT ChannelFactory::Instance()->Init duration_ms=" + std::to_string(ms),
            "init");

        g_last_successful_phase = "PHASE 1: ChannelFactory::Init";
        SleepMs(kPhaseInitDelayMs);
    }

    // ------------------------------------------------------------------------
    // PHASE 2: B1LocoClient::Init
    // ------------------------------------------------------------------------
    B1LocoClient loco;
    {
        ScopedPhaseTimer phase("PHASE 2: B1LocoClient::Init");

        LogMsg(LogLevel::kInfo, "ENTER loco.Init()", "init");
        const auto start = std::chrono::steady_clock::now();
        loco.Init();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        LogMsg(LogLevel::kInfo, "EXIT loco.Init() duration_ms=" + std::to_string(ms), "init");

        g_last_successful_phase = "PHASE 2: B1LocoClient::Init";
        SleepMs(kPhaseInitDelayMs);
    }

    // ------------------------------------------------------------------------
    // PHASE 3: vision bridge file reader init (replaces VisionClient::Init)
    // ------------------------------------------------------------------------
    VisionFileReader vision_reader(kVisionBridgeFilePath);
    {
        ScopedPhaseTimer phase("PHASE 3: vision bridge file reader init");
        LogMsg(LogLevel::kInfo,
            "Will read detections from " + std::string(kVisionBridgeFilePath) +
                " (written by the separate vision_bridge_node process).",
            "init");
        g_last_successful_phase = "PHASE 3: vision bridge file reader init";
        SleepMs(kPhaseInitDelayMs);
    }

    // ------------------------------------------------------------------------
    // PHASE 4: ChangeMode(kWalking)
    // ------------------------------------------------------------------------
    {
        ScopedPhaseTimer phase("PHASE 4: ChangeMode(kWalking)");

        if (RetryRpc("loco.ChangeMode(kWalking)", [&]() -> int {
                return loco.ChangeMode(booster::robot::RobotMode::kWalking);
            }) != 0) {
            LogMsg(LogLevel::kError,
                "PHASE 4 failed. This means locomotion mode switch is a likely timeout source.",
                "init");
            EnterSafeState(loco);
            PrintFailureSummary(State::kSearchBall);
            return 1;
        }

        g_walking_mode_succeeded = true;
        g_last_successful_phase = "PHASE 4: ChangeMode(kWalking)";
        SleepMs(kPhaseModeDelayMs);
    }

    // ------------------------------------------------------------------------
    // PHASE 5: wait for the vision bridge file to actually have data
    // ------------------------------------------------------------------------
    if (!WaitForVisionTopic(vision_reader)) {
        EnterSafeState(loco);
        PrintFailureSummary(State::kSearchBall);
        return 1;
    }
    g_vision_started_successfully = true;
    g_last_successful_phase = "PHASE 5: wait for vision bridge file";

    // ------------------------------------------------------------------------
    // PHASE 6: state machine start
    // ------------------------------------------------------------------------
    {
        ScopedPhaseTimer phase("PHASE 6: state machine start");
        g_last_successful_phase = "PHASE 6: state machine start";
    }

    State state = State::kSearchBall;
    std::vector<float> person_pos;
    float ball_x = 0.35f;
    float ball_y = 0.0f;
    auto last_ball_reading_time = std::chrono::steady_clock::now();
    // Plausibility baseline for APPROACH_BALL (see the constants above for
    // the full reasoning). -1 means "not yet established"; SEARCH_BALL sets
    // this from an averaged multi-sample reading whenever the ball is found.
    float ball_baseline_range = -1.0f;
    int implausible_far_count = 0;
    int sweep_dir = 1;
    int sweep_count = 0;
    int detection_rpc_failures = 0;
    int ball_lost_count = 0;
    int person_lost_count = 0;
    int loop_count = 0;

    LogMsg(LogLevel::kInfo, "Initial state SEARCH_BALL", "state");

    while (g_run.load()) {
        ++loop_count;

        // Heartbeat log so we can see whether the loop is progressing normally.
        if (loop_count % kHeartbeatEveryLoops == 0) {
            std::ostringstream hb;
            hb << "loop=" << loop_count
               << " state=" << StateName(state)
               << " ball=" << FormatXY(ball_x, ball_y)
               << " person=" << FormatVecXY(person_pos)
               << " detection_rpc_failures=" << detection_rpc_failures
               << " ball_lost_count=" << ball_lost_count
               << " person_lost_count=" << person_lost_count
               << " sweep_dir=" << sweep_dir
               << " sweep_count=" << sweep_count;
            LogMsg(LogLevel::kInfo, hb.str(), "heartbeat");
        }

        // Note: the SDK path's per-request "ratio" (narrowing focus to the
        // central 33% of frame during APPROACH_BALL, to reduce false matches)
        // has no equivalent here - vision_node.cpp always publishes full-frame
        // detections over the topic, so that narrowing is dropped rather than
        // reimplemented client-side.
        std::vector<SimpleDetection> objects;

        if (!PollDetections(vision_reader, objects)) {
            ++detection_rpc_failures;

            LogMsg(LogLevel::kWarn,
                "Detection poll failed (missing/stale vision bridge file). state=" + std::string(StateName(state)) +
                    " detection_rpc_failures=" + std::to_string(detection_rpc_failures),
                "vision");

            if (state != State::kSearchBall && state != State::kKick &&
                detection_rpc_failures >= kActivePollFailureLimit) {
                RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                    ball_lost_count, person_lost_count,
                                    person_pos,
                                    "repeated detection poll failure during active state");
            }

            SleepMs(PollDelayMsForState(state));
            continue;
        }

        detection_rpc_failures = 0;

        const SimpleDetection* best_ball = nullptr;
        const SimpleDetection* best_person = nullptr;

        for (const auto& object : objects) {
            if (IsBall(object.tag_) && HasRobotFrameXY(object)) {
                if (!best_ball || object.conf_ > best_ball->conf_) {
                    best_ball = &object;
                }
            } else if (IsPerson(object.tag_) && HasRobotFrameXY(object)) {
                if (!best_person || object.conf_ > best_person->conf_) {
                    best_person = &object;
                }
            }
        }

        switch (state) {
            case State::kSearchBall: {
                if (best_ball) {
                    StopMotion(loco);

                    // Average several readings instead of trusting the single
                    // frame that first crossed the detection threshold, so
                    // both the initial ball_x/ball_y and the plausibility
                    // baseline below start from a robust value rather than
                    // one potentially noisy sample.
                    float sum_x = 0.0f;
                    float sum_y = 0.0f;
                    int sample_count = 0;
                    for (int i = 0; i < kBaselineSampleCount; ++i) {
                        if (i > 0) {
                            SleepMs(kBaselineSampleIntervalMs);
                        }
                        std::vector<SimpleDetection> baseline_objects;
                        if (!PollDetections(vision_reader, baseline_objects)) {
                            continue;
                        }
                        const SimpleDetection* sample = FindBestBall(baseline_objects);
                        if (!sample) {
                            continue;
                        }
                        float sample_x = sample->position_[0];
                        float sample_y = sample->position_[1];
                        CompensateForLatency(sample_x, sample_y, vision_reader.LastPipelineLatencyMs());
                        sum_x += sample_x;
                        sum_y += sample_y;
                        ++sample_count;
                    }

                    if (sample_count > 0) {
                        ball_x = sum_x / static_cast<float>(sample_count);
                        ball_y = sum_y / static_cast<float>(sample_count);
                    } else {
                        // Every follow-up sample missed (e.g. the ball
                        // slipped out of view again immediately) - fall back
                        // to the original single reading rather than
                        // dividing by zero.
                        ball_x = best_ball->position_[0];
                        ball_y = best_ball->position_[1];
                        CompensateForLatency(ball_x, ball_y, vision_reader.LastPipelineLatencyMs());
                    }
                    last_ball_reading_time = std::chrono::steady_clock::now();
                    ball_baseline_range = Dist2D({ball_x, ball_y});
                    implausible_far_count = 0;

                    LogMsg(LogLevel::kInfo,
                        "Best ball found at " + FormatXY(ball_x, ball_y) +
                            " (averaged " + std::to_string(sample_count) + " samples)" +
                            " baseline_range=" + std::to_string(ball_baseline_range),
                        "vision");

                    ball_lost_count = 0;
                    person_lost_count = 0;
                    sweep_dir = 1;
                    sweep_count = 0;

                    const State old_state = state;
                    state = State::kApproachBall;
                    LogStateTransition(old_state, state, "ball found");
                    break;
                }

                (void)LoggedRotateHeadWithDirection(loco, sweep_dir, 0, "searching for ball");
                ++sweep_count;

                if (sweep_count >= kSweepStepsHalf * 2) {
                    sweep_dir = -sweep_dir;
                    sweep_count = 0;

                    LogMsg(LogLevel::kDebug,
                        "Full ball-search head sweep completed; rotating body slightly.",
                        "search");

                    (void)LoggedMove(loco, 0.0f, 0.0f,
                                     static_cast<float>(sweep_dir) * kBallSearchBodyYawRate,
                                     "widen ball search field");
                    SleepMs(kSearchBodyRotateMs);
                    (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "stop post-search rotate");
                }
                break;
            }

            case State::kApproachBall: {
                if (!best_ball) {
                    ++ball_lost_count;
                    LogMsg(LogLevel::kWarn,
                        "Ball not visible while approaching. ball_lost_count=" +
                            std::to_string(ball_lost_count),
                        "state");

                    if (ball_lost_count >= kTargetLossLimit) {
                        RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                            ball_lost_count, person_lost_count,
                                            person_pos,
                                            "ball lost while approaching");
                    }
                    break;
                }

                ball_lost_count = 0;
                {
                    const auto now_reading = std::chrono::steady_clock::now();
                    const double dt_s = std::chrono::duration<double>(
                        now_reading - last_ball_reading_time).count();

                    // Raw per-frame readings swing by 10-30cm frame to frame
                    // even under normal conditions (confirmed from logged
                    // approach data: 1.043 -> 1.116 -> 0.987 -> 0.948m across
                    // ~250ms while walking in a straight line) - that alone
                    // implies several m/s, well above the robot's real
                    // ~0.2 m/s top speed. A hard accept/reject filter based
                    // on implied speed (tried previously) couldn't tell
                    // ordinary noise apart from a genuinely bad reading,
                    // since both look "too fast" by that measure - it ended
                    // up rejecting almost everything and got stuck comparing
                    // against an increasingly stale reference, which caused
                    // the robot to lose the ball entirely in testing.
                    //
                    // Rate-limit the *output* instead of rejecting the
                    // *input*: blend the raw reading in via the usual EMA,
                    // then cap how far that result is allowed to move from
                    // the previous estimate this cycle. This always makes
                    // progress toward the latest data (so it can't get stuck
                    // the way the reject filter did) while still bounding
                    // how much any single extreme outlier can yank the
                    // controller's belief in one step.
                    float latency_compensated_x = best_ball->position_[0];
                    float latency_compensated_y = best_ball->position_[1];
                    CompensateForLatency(latency_compensated_x, latency_compensated_y,
                                         vision_reader.LastPipelineLatencyMs());

                    // Plausibility check: reject readings implying the ball
                    // is farther than the best (closest) distance seen so
                    // far, unless a farther reading persists for several
                    // consecutive polls (escape hatch - see the constants
                    // above for why a hard, permanent reject isn't safe).
                    const float compensated_range =
                        Dist2D({latency_compensated_x, latency_compensated_y});
                    bool accept_reading = true;
                    if (ball_baseline_range >= 0.0f &&
                        compensated_range > ball_baseline_range + kPlausibilityMarginM) {
                        ++implausible_far_count;
                        if (implausible_far_count < kPlausibilityEscapeHatchPolls) {
                            accept_reading = false;
                            LogMsg(LogLevel::kDebug,
                                "Ball reading rejected as implausibly far: range=" +
                                    std::to_string(compensated_range) + " baseline=" +
                                    std::to_string(ball_baseline_range) + " count=" +
                                    std::to_string(implausible_far_count),
                                "vision");
                        } else {
                            LogMsg(LogLevel::kInfo,
                                "Implausibly-far ball reading persisted " +
                                    std::to_string(implausible_far_count) +
                                    " polls - accepting as real, moving baseline " +
                                    std::to_string(ball_baseline_range) + " -> " +
                                    std::to_string(compensated_range),
                                "vision");
                            ball_baseline_range = compensated_range;
                            implausible_far_count = 0;
                        }
                    } else {
                        implausible_far_count = 0;
                        if (ball_baseline_range < 0.0f || compensated_range < ball_baseline_range) {
                            ball_baseline_range = compensated_range;
                        }
                    }

                    if (accept_reading) {
                        const float candidate_x =
                            kBallPositionSmoothingAlpha * latency_compensated_x +
                            (1.0f - kBallPositionSmoothingAlpha) * ball_x;
                        const float candidate_y =
                            kBallPositionSmoothingAlpha * latency_compensated_y +
                            (1.0f - kBallPositionSmoothingAlpha) * ball_y;

                        const double step = std::hypot(
                            static_cast<double>(candidate_x) - ball_x,
                            static_cast<double>(candidate_y) - ball_y);
                        const double max_step = kMaxBallTrackingSpeedMps * dt_s;

                        if (max_step > 1e-6 && step > max_step) {
                            const double scale = max_step / step;
                            ball_x = static_cast<float>(ball_x + (candidate_x - ball_x) * scale);
                            ball_y = static_cast<float>(ball_y + (candidate_y - ball_y) * scale);
                            LogMsg(LogLevel::kDebug,
                                "Ball tracking step capped: wanted " + std::to_string(step) +
                                    "m over dt=" + std::to_string(dt_s) + "s, capped to " +
                                    std::to_string(max_step) + "m -> " + FormatXY(ball_x, ball_y),
                                "vision");
                        } else {
                            ball_x = candidate_x;
                            ball_y = candidate_y;
                        }
                        last_ball_reading_time = now_reading;
                    }
                }

                const double ex = ball_x - kBallReadyDistMin;
                const double ey = ball_y;
                // Gain history: 0.8 -> 0.25 -> 0.1, progressively widening
                // the deceleration zone to fix running into the ball. Wrong
                // fix: at gain=0.1, most of the approach commanded vx in the
                // 0.05-0.12 m/s range, which turned out to be at or below
                // this robot's minimum walkable gait speed - confirmed by
                // watching it: it moved briefly, then just rocked in place
                // for the rest of a 2-minute run without meaningfully
                // closing distance. A wheeled robot can glide down to near
                // zero; this one apparently can't take clean steps that
                // slow. robocup_demo's own RobotClient::setVelocity has
                // exactly this problem already solved: it floors any
                // non-zero velocity command at a minimum (get_min_vx())
                // rather than letting it decay smoothly. Doing the same
                // here - keep a solid, reliably-walkable cruise speed, and
                // rely on the widened ready-range margin + smoothed ball
                // position + HardStopAndSettle for the actual stop, instead
                // of asking the gait to finely decelerate (which it can't).
                double vx = std::clamp(0.4 * ex, -0.20, 0.20);
                if (ex > kApproachVxMinZoneThreshold && vx < kApproachVxMin) {
                    vx = kApproachVxMin;
                }
                const double vy = std::clamp(1.0 * ey, -0.12, 0.12);
                const double wz = std::clamp(1.2 * ey, -0.4, 0.4);

                LogMsg(LogLevel::kDebug,
                    "Approach controller ball=" + FormatXY(ball_x, ball_y) +
                        " ex=" + std::to_string(ex) +
                        " ey=" + std::to_string(ey) +
                        " vx=" + std::to_string(vx) +
                        " vy=" + std::to_string(vy) +
                        " wz=" + std::to_string(wz) +
                        " pipeline_latency_ms=" + std::to_string(vision_reader.LastPipelineLatencyMs()),
                    "control");

                (void)LoggedMove(loco,
                                 static_cast<float>(vx),
                                 static_cast<float>(vy),
                                 static_cast<float>(wz),
                                 "approach ball");

                if (ball_x > kBallReadyDistMin &&
                    ball_x < kBallReadyDistMax &&
                    std::abs(ball_y) < kBallAlignY) {
                    HardStopAndSettle(loco);
                    person_lost_count = 0;
                    sweep_dir = 1;
                    sweep_count = 0;

                    const State old_state = state;
                    state = State::kSearchPerson;
                    LogStateTransition(old_state, state, "ball in ready range");
                }
                break;
            }

            case State::kSearchPerson: {
                if (best_person) {
                    person_pos = best_person->position_;
                    person_lost_count = 0;

                    LogMsg(LogLevel::kInfo,
                        "Person locked at " + FormatVecXY(person_pos),
                        "vision");

                    StopMotion(loco);

                    const State old_state = state;
                    state = State::kAlignBody;
                    LogStateTransition(old_state, state, "person locked");
                    break;
                }

                (void)LoggedRotateHeadWithDirection(loco, sweep_dir, 0, "searching for person");
                ++sweep_count;

                if (sweep_count >= kSweepStepsHalf * 2) {
                    sweep_dir = -sweep_dir;
                    sweep_count = 0;

                    LogMsg(LogLevel::kDebug,
                        "Full person-search head sweep completed; rotating body slightly.",
                        "search");

                    (void)LoggedMove(loco, 0.0f, 0.0f,
                                     static_cast<float>(sweep_dir) * kPersonSearchBodyYawRate,
                                     "widen person search field");
                    SleepMs(kSearchBodyRotateMs);
                    (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "stop post-search rotate");
                }
                break;
            }

            case State::kAlignBody: {
                if (best_person) {
                    person_pos = best_person->position_;
                    person_lost_count = 0;
                    LogMsg(LogLevel::kDebug,
                        "Refresh person target " + FormatVecXY(person_pos),
                        "vision");
                } else {
                    ++person_lost_count;
                    LogMsg(LogLevel::kWarn,
                        "Person not visible while aligning. person_lost_count=" +
                            std::to_string(person_lost_count),
                        "state");

                    if (person_lost_count >= kTargetLossLimit) {
                        RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                            ball_lost_count, person_lost_count,
                                            person_pos,
                                            "person lost while aligning");
                    }
                    break;
                }

                const float angle = std::atan2(
                    person_pos[1],
                    std::max(person_pos[0], kMinPersonFwdDist));

                LogMsg(LogLevel::kDebug,
                    "Alignment angle=" + std::to_string(angle) +
                        " threshold=" + std::to_string(kAlignYawThresh),
                    "control");

                if (std::abs(angle) > kAlignYawThresh) {
                    (void)LoggedMove(loco, 0.0f, 0.0f,
                                     std::copysign(kAlignBodyYawRate, angle),
                                     "align body to person");
                } else {
                    (void)LoggedMove(loco, 0.0f, 0.0f, 0.0f, "alignment complete");
                    const State old_state = state;
                    state = State::kKick;
                    g_reached_kick_state = true;
                    LogStateTransition(old_state, state, "body aligned");
                }
                break;
            }

            case State::kKick: {
                StopMotionAndCenterHead(loco);

                if (person_pos.size() < 2) {
                    RecoverToSearchBall(loco, state, sweep_dir, sweep_count,
                                        ball_lost_count, person_lost_count,
                                        person_pos,
                                        "kick state reached without valid person target");
                    break;
                }

                const float distance = Dist2D(person_pos);
                const float power = std::clamp(
                    kPowerMin + (kPowerMax - kPowerMin) * (distance / kPowerDist),
                    kPowerMin,
                    kPowerMax);

                LogMsg(LogLevel::kInfo,
                    "Kick prep person=" + FormatVecXY(person_pos) +
                        " distance=" + std::to_string(distance) +
                        " power=" + std::to_string(power),
                    "kick");

                if (RetryRpc("loco.ChangeMode(kSoccer)", [&]() -> int {
                        return loco.ChangeMode(booster::robot::RobotMode::kSoccer);
                    }) != 0) {
                    LogMsg(LogLevel::kError,
                        "ChangeMode(kSoccer) failed. If timeout appears here, the issue is "
                        "specific to soccer mode service readiness.",
                        "kick");
                    EnterSafeState(loco);
                    g_run = false;
                    break;
                }

                SleepMs(kPhaseModeDelayMs);

                if (RetryRpc("loco.VisualKick(true)", [&]() -> int {
                        return loco.VisualKick(true, VisualKickVersion::kV2);
                    }) != 0) {
                    LogMsg(LogLevel::kError,
                        "VisualKick(true) failed. This identifies kick-start RPC as the likely source.",
                        "kick");
                    EnterSafeState(loco);
                    g_run = false;
                    break;
                }

                // Publishes continuously for up to kKickMotionWaitMs, re-polling
                // live ball position each cycle and aborting early if the ball
                // moves or goes stale - see PublishKickFrames for the reasoning.
                // No separate blind sleep needed afterward: the loop itself
                // spans the full kick execution window.
                const bool published = PublishKickFrames(loco, vision_reader, ball_x, ball_y, person_pos, power);

                (void)RetryRpc("loco.VisualKick(false)", [&]() -> int {
                    return loco.VisualKick(false, VisualKickVersion::kV2);
                }, 2);

                if (!published) {
                    LogMsg(LogLevel::kError, "Failed to publish kick reference frames.", "kick");
                } else {
                    LogMsg(LogLevel::kInfo, "Pass complete.", "kick");
                }

                EnterSafeState(loco);
                g_run = false;
                break;
            }
        }

        SleepMs(PollDelayMsForState(state));
    }

    LogMsg(LogLevel::kInfo, "Main loop exited; beginning shutdown.", "main");
    EnterSafeState(loco);

    PrintFailureSummary(state);
    return 0;
}