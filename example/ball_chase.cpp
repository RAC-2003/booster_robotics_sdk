// ball_chase.cpp
//
// Behavior: find the ball, walk up to it, stop.
//
// Vision: this reads /dev/shm/booster_vision_bridge.txt instead of calling
// the SDK's VisionClient RPC directly. That file is written by a separate
// process, vision_bridge_node (vision_ros_bridge_pkg/src/vision_bridge_node.cpp),
// which subscribes to robocup_demo's own vision output - the
// /booster_soccer/detection topic published by vision_node - and relays it
// to plain text so a non-ROS2 program can read it. The data itself (label,
// confidence, position) is robocup_demo's real vision_node output,
// unmodified; the bridge is just a relay, needed because ROS2 and the
// Booster SDK each require an incompatible version of the same underlying
// networking library (FastDDS) and can't be loaded in one process
// (vision_bridge_node.cpp's own header comment has the full explanation).
// The SDK's VisionClient RPC path was tried directly first, but is
// unverified/previously reported not to complete its handshake on this
// robot - this file reader is the path with actual on-robot precedent.
//
// Everything downstream of "what does the ball selection/search/approach
// logic look like" is still ported only from the SDK or robocup_demo:
//   - Ball selection (label match, confidence threshold, sanity bounds on
//     x) mirrors Brain::getGameObjects/detectProcessBalls in
//     robocup_demo/src/brain/src/brain.cpp.
//   - Head-sweep search mirrors the CamFindBall behavior-tree node in
//     robocup_demo/src/brain/src/brain_tree.cpp.
//   - Steering while approaching (lateral + facing the ball) mirrors the
//     SimpleChase behavior-tree node (same file); forward speed is a flat,
//     directly-set constant (kApproachSpeed) instead of SimpleChase's
//     distance-proportional vx, by request. Both are fed through the same
//     floor/cap velocity logic as RobotClient::setVelocity in
//     robocup_demo/src/brain/src/robot_client.cpp.
//   - Numeric limits (vx_limit, min_vx, ball_confidence_threshold, ...)
//     come from robocup_demo/src/brain/config/config.yaml.
//   - Motion (Move/RotateHead/ChangeMode) is the Booster SDK's B1LocoClient.
//
// Requires vision_node + vision_bridge_node already running (ball_pass/
// startup.sh brings both up) before this program will see any detections.
//
// Usage:
//   cd ~/Workspace/ball_pass/build && ./ball_chase [network_interface]
//   (defaults to "lo")

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <booster/robot/b1/b1_loco_api.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_factory.hpp>

using namespace std::chrono_literals;
using booster::robot::ChannelFactory;
using booster::robot::RobotMode;
using booster::robot::b1::B1LocoClient;

// ============================================================================
// Block 1: constants
// ============================================================================
// Where the bridge file lives, and how old a reading is allowed to be
// before it's treated as stale (guards against reading a frozen file left
// behind by a dead vision_bridge_node). Both are properties of the bridge
// mechanism itself, not values taken from the SDK or robocup_demo.
static constexpr const char *kVisionBridgeFilePath = "/dev/shm/booster_vision_bridge.txt";
static constexpr long long kMaxDetectionAgeMs = 500;

// Global velocity limits and floors, from config.yaml's "strategy" section
// (vx_limit/vy_limit/vtheta_limit/min_vx/min_vy/min_vtheta) - the same
// numbers RobotClient::setVelocity() applies to every motion command.
static constexpr double kGlobalVxLimit = 1.0;
static constexpr double kGlobalVyLimit = 0.4;
static constexpr double kGlobalVthetaLimit = 1.2;
static constexpr double kMinVx = 0.3;      // setVelocity floors any nonzero
static constexpr double kMinVy = 0.3;      // command below this, "to prevent
static constexpr double kMinVtheta = 0.25; // no response" (config.yaml comment)

// Constant forward approach speed (m/s), set directly here - unlike
// SimpleChase's distance-proportional vx, this does not slow down as the
// ball gets closer, it just walks at this speed until within kStopDist,
// then stops. Change this one number to change how fast it walks up.
static constexpr double kApproachSpeed = 0.3;

// Lateral steering still tracks the ball's position, so the robot actually
// walks toward the ball instead of straight ahead regardless of where it
// is - this cap is SimpleChase's own vy_limit from subtree_striker_play.xml.
static constexpr double kApproachVyLimit = 0.2;
static constexpr double kApproachVthetaGain = 4.0; // SimpleChase: vtheta = ball.yawToRobot * 4.0

// Ball selection, from Brain::detectProcessBalls/getGameObjects in brain.cpp:
// confidence threshold from config.yaml, and the x-range sanity check
// ("prevent misidentifying lights in the sky as balls"). The bridge file's
// confidence field is the same vision_interface::msg::Detections field
// brain.cpp compares against this threshold, so the scale matches exactly.
static constexpr float kBallConfidenceThreshold = 50.0f;
static constexpr double kBallXMin = -0.5;
static constexpr double kBallXMax = 15.0;

// How long the ball can go undetected before we give up and go back to
// searching, from config.yaml's ball_memory_timeout (used in brain.cpp to
// delay data->lose_ball so one missed frame doesn't cause a loop).
static constexpr long long kBallMemoryTimeoutMs = 2000;

// SimpleChase stops translating once ball.range < stop_dist. SimpleChase
// itself is only ever invoked in robocup_demo with stop_dist=0.0 (for
// assist_chase, which is meant to keep following, never actually stop) so
// there's no single "correct" production value to copy for "walk up and
// stop". 0.5m is chosen here, informed by the safe_dist=0.5 standoff used
// for the real ball-approach Chase node in subtree_striker_play.xml.
static constexpr double kStopDist = 1.0;

// RLVisionKick::stepDecelerate (brain_tree.cpp) holds a zero-velocity
// command for startDecelerate(500.0) - 500ms - before switching modes, to
// let the gait actually come to rest rather than just zeroing the target
// velocity. Reused here as the hold time once we've arrived at the ball.
static constexpr int kDoneHoldMs = 500;
static constexpr int kDoneHoldStepMs = 100;

// CamFindBall's head-sweep table and timing (brain_tree.cpp constructor):
// six fixed pitch/yaw poses, advanced one step per second.
static constexpr double kSearchLowPitch = 1.0;
static constexpr double kSearchHighPitch = 0.2;
static constexpr double kSearchLeftYaw = 1.1;
static constexpr double kSearchRightYaw = -1.1;
static constexpr int kSearchCmdIntervalMs = 1000;
static const double kSearchSequence[6][2] = {
    {kSearchLowPitch, kSearchLeftYaw},
    {kSearchLowPitch, 0.0},
    {kSearchLowPitch, kSearchRightYaw},
    {kSearchHighPitch, kSearchRightYaw},
    {kSearchHighPitch, 0.0},
    {kSearchHighPitch, kSearchLeftYaw},
};

// Once the ball is this close during APPROACH, tilt the head down to a
// fixed pose to keep looking at it near the robot's feet rather than out
// ahead - a flat, directly-set value (same pitch CamFindBall itself uses
// for its near/low-angle search pose), not calculated from distance.
static constexpr double kApproachCloseRangeDist = 1.0;
static constexpr double kApproachCloseHeadPitch = kSearchLowPitch;

// Head pose the robot returns to once stopped (kDone) - level/up, yaw
// centered.
static constexpr double kHeadUpPitch = 0.0;

// Not derived from either source - just how often this (non-ROS, blocking
// loop) program polls vision and re-evaluates state, since there's no
// behavior-tree tick rate to inherit here.
static constexpr int kPollIntervalMs = 100;

// ============================================================================
// Block 2: vision bridge file reader
// ============================================================================
// Mirrors the file format documented in vision_bridge_node.cpp:
//   line 1: "<write_ts_ms> <capture_ts_ms>"
//   line 2+: one detection per line, "label|confidence|x,y,z"
struct BridgeDetection {
    std::string label;
    float confidence = 0.0f;
    std::vector<float> position;
};

class VisionFileReader {
public:
    explicit VisionFileReader(std::string path) : path_(std::move(path)) {}

    // Returns true only if the file exists, parses, and was written within
    // kMaxDetectionAgeMs - guards against reading stale data from a dead
    // vision_bridge_node.
    bool Poll(std::vector<BridgeDetection> &out) {
        out.clear();

        std::ifstream in(path_);
        if (!in.is_open()) {
            return false;
        }

        long long write_ts_ms = 0;
        in >> write_ts_ms;
        std::string discard;
        std::getline(in, discard); // consume rest of the timestamp line

        std::string line;
        while (std::getline(in, line)) {
            if (line.empty()) continue;

            const size_t p1 = line.find('|');
            const size_t p2 = (p1 == std::string::npos) ? std::string::npos : line.find('|', p1 + 1);
            if (p1 == std::string::npos || p2 == std::string::npos) continue;

            BridgeDetection d;
            d.label = line.substr(0, p1);
            try {
                d.confidence = std::stof(line.substr(p1 + 1, p2 - p1 - 1));
            } catch (const std::exception &) {
                continue;
            }

            std::stringstream pos_stream(line.substr(p2 + 1));
            std::string value;
            while (std::getline(pos_stream, value, ',')) {
                if (value.empty()) continue;
                try {
                    d.position.push_back(std::stof(value));
                } catch (const std::exception &) {
                    // skip malformed field, keep the rest of the detection
                }
            }
            out.push_back(std::move(d));
        }

        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();
        return (now_ms - write_ts_ms) <= kMaxDetectionAgeMs;
    }

private:
    std::string path_;
};

// ============================================================================
// Block 3: small helper, ported from utils/math.h
// ============================================================================
static double Cap(double x, double upper, double lower) {
    return std::max(std::min(x, upper), lower);
}

// ============================================================================
// Block 4: ball selection, ported from Brain::getGameObjects +
// Brain::detectProcessBalls (brain.cpp)
// ============================================================================
struct Ball {
    double x = 0.0;          // posToRobot.x: forward distance, meters
    double y = 0.0;          // posToRobot.y: lateral offset, meters
    double range = 0.0;      // norm(x, y)
    double yawToRobot = 0.0; // atan2(y, x)
};

static bool FindBestBall(const std::vector<BridgeDetection> &objects, Ball &out) {
    const BridgeDetection *best = nullptr;
    for (const auto &obj : objects) {
        if (obj.label != "Ball") continue; // brain.cpp only ever matches "Ball" exactly
        if (obj.position.size() < 2) continue;
        if (obj.confidence < kBallConfidenceThreshold) continue;

        const double x = obj.position[0];
        if (x < kBallXMin || x > kBallXMax) continue;

        if (!best || obj.confidence > best->confidence) {
            best = &obj;
        }
    }

    if (!best) return false;

    out.x = best->position[0];
    out.y = best->position[1];
    out.range = std::sqrt(out.x * out.x + out.y * out.y);
    out.yawToRobot = std::atan2(out.y, out.x);
    return true;
}

// ============================================================================
// Block 5: motion, ported from RobotClient::setVelocity (robot_client.cpp)
// ============================================================================
// setVelocity floors any nonzero command below the configured minimum (so
// the gait doesn't just ignore a too-small request), then caps to the
// global limits, before actually sending the move.
static void SetVelocity(B1LocoClient &loco, double x, double y, double theta) {
    if (std::fabs(x) < kMinVx && std::fabs(x) > 1e-5) x = x > 0 ? kMinVx : -kMinVx;
    if (std::fabs(y) < kMinVy && std::fabs(y) > 1e-5) y = y > 0 ? kMinVy : -kMinVy;
    if (std::fabs(theta) < kMinVtheta && std::fabs(theta) > 1e-5) theta = theta > 0 ? kMinVtheta : -kMinVtheta;

    x = Cap(x, kGlobalVxLimit, -kGlobalVxLimit);
    y = Cap(y, kGlobalVyLimit, -kGlobalVyLimit);
    theta = Cap(theta, kGlobalVthetaLimit, -kGlobalVthetaLimit);

    loco.Move(static_cast<float>(x), static_cast<float>(y), static_cast<float>(theta));
}

// ============================================================================
// Block 6: Ctrl+C handling
// ============================================================================
static std::atomic<bool> g_run{true};
static void OnSigint(int) { g_run = false; }

// ============================================================================
// Block 7: main
// ============================================================================
int main(int argc, char **argv) {
    std::signal(SIGINT, OnSigint);

    const std::string iface = (argc > 1) ? argv[1] : "lo";
    std::cout << "ball_chase - detect ball, walk up to it, stop. "
                 "(network interface: " << iface << ", Ctrl+C to abort)\n";
    std::cout << "Reading detections from " << kVisionBridgeFilePath
              << " - make sure ball_pass/startup.sh has already been run.\n";

    ChannelFactory::Instance()->Init(0, iface);

    B1LocoClient loco;
    loco.Init();
    std::this_thread::sleep_for(2s);

    VisionFileReader vision_reader(kVisionBridgeFilePath);

    std::cout << "Ensure the robot is standing upright with clear space ahead. "
                 "Press Enter to start walking, Ctrl+C to abort: ";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if (!g_run) return 0;

    int rc = loco.ChangeMode(RobotMode::kWalking); // robocupWalk()
    if (rc != 0) {
        std::cout << "[FAIL] ChangeMode(kWalking) rc=" << rc << ". Aborting.\n";
        return 1;
    }
    std::this_thread::sleep_for(1s);

    enum class State { kSearch, kApproach, kDone };
    State state = State::kSearch;

    // CamFindBall's own bookkeeping: which pose in the sweep table is
    // active, and when it last advanced.
    int search_index = 0;
    auto last_search_cmd = std::chrono::steady_clock::now();

    // Tracks whether the fixed close-range head pose has already been sent,
    // so it's a one-time command, not repeated every poll while close.
    bool head_tilted_down = false;

    // detectProcessBalls' own bookkeeping: last time a real ball was seen,
    // used to give the ball a grace period (kBallMemoryTimeoutMs) before
    // treating it as truly lost.
    Ball ball;
    bool ball_ever_seen = false;
    std::chrono::steady_clock::time_point last_seen_time;

    while (g_run && state != State::kDone) {
        std::vector<BridgeDetection> objects;
        vision_reader.Poll(objects);

        Ball fresh_ball;
        const bool found = FindBestBall(objects, fresh_ball);
        const auto now = std::chrono::steady_clock::now();
        if (found) {
            ball = fresh_ball;
            ball_ever_seen = true;
            last_seen_time = now;
        }
        const long long since_seen_ms = ball_ever_seen
            ? std::chrono::duration_cast<std::chrono::milliseconds>(now - last_seen_time).count()
            : -1;
        const bool ball_location_known = ball_ever_seen && since_seen_ms <= kBallMemoryTimeoutMs;

        switch (state) {
            case State::kSearch: {
                if (ball_location_known) {
                    std::cout << "[SEARCH] ball found at (" << ball.x << ", " << ball.y << ") -> APPROACH\n";
                    state = State::kApproach;
                    break;
                }

                // CamFindBall::tick(): advance to the next head pose once
                // per kSearchCmdIntervalMs, otherwise leave the head alone.
                const long long since_cmd_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                    now - last_search_cmd)
                                                    .count();
                if (since_cmd_ms >= kSearchCmdIntervalMs) {
                    search_index = (search_index + 1) % 6;
                    loco.RotateHead(kSearchSequence[search_index][0], kSearchSequence[search_index][1]);
                    last_search_cmd = now;
                }
                break;
            }

            case State::kApproach: {
                if (!ball_location_known) {
                    std::cout << "[APPROACH] ball lost -> back to SEARCH\n";
                    SetVelocity(loco, 0.0, 0.0, 0.0);
                    state = State::kSearch;
                    break;
                }

                // Constant-speed approach: walk at kApproachSpeed while
                // steering to keep facing/centering the ball (vy, vtheta),
                // instead of SimpleChase's distance-proportional vx.
                double vx = kApproachSpeed;
                double vy = Cap(ball.y, kApproachVyLimit, -kApproachVyLimit);
                double vtheta = ball.yawToRobot * kApproachVthetaGain;

                if (ball.range <= kApproachCloseRangeDist && !head_tilted_down) {
                    loco.RotateHead(kApproachCloseHeadPitch, 0.0);
                    head_tilted_down = true;
                }

                if (ball.range < kStopDist) {
                    vx = 0.0;
                    vy = 0.0;
                }

                SetVelocity(loco, vx, vy, vtheta);

                if (ball.range < kStopDist) {
                    std::cout << "[APPROACH] arrived at ball, range=" << ball.range << " -> stopping\n";
                    for (int elapsed = 0; elapsed < kDoneHoldMs; elapsed += kDoneHoldStepMs) {
                        SetVelocity(loco, 0.0, 0.0, 0.0);
                        std::this_thread::sleep_for(std::chrono::milliseconds(kDoneHoldStepMs));
                    }
                    loco.RotateHead(kHeadUpPitch, 0.0);
                    state = State::kDone;
                }
                break;
            }

            case State::kDone:
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));
    }

    if (state != State::kDone) {
        SetVelocity(loco, 0.0, 0.0, 0.0);
        std::cout << "\n[ABORTED]\n";
        return 0;
    }

    std::cout << "\n[DONE] stopped at the ball.\n";
    return 0;
}
