// path_follower.cpp
//
// Step 1 (odometry block): read /odom and know the robot's current x, y, yaw.
//
// Step 2 (waypoint block): load a taught waypoint CSV, and each control
// cycle pick which waypoint ahead of the robot we're currently aiming at
// (doc section 2.1-2.2: forward-only search from the last picked index for
// the first waypoint at least lookahead_m away).
//
// Step 3 (steering block): turn that target point into v_cmd/w_cmd (doc
// sections 2.3-2.5 and pseudocode steps D/E) - rotate the target into the
// robot's own frame, then curvature kappa = 2*y_r/L_d^2 says how hard to
// turn toward it.
//
// Step 4 (safety block): Rule 2 (heading protection), Rule 3 (clamp v/w to
// safe limits), Rule 4 (goal stop). Rule 1 lives in the waypoint block.
//
// Step 5 (movement block): two changes together.
//   a) The control cycle moved from "runs on every /odom message" (500+Hz,
//      fine for printing, wrong for commanding the robot - doc section 5
//      recommends 20-30Hz) to a fixed-rate timer. The /odom callback now
//      just caches the latest reading; the timer does the actual work.
//      This also makes Rule 5 possible: staleness can only be detected by
//      something running on a clock independent of whether new odometry
//      arrives, not from inside the callback that only fires when it does.
//   b) v_cmd/w_cmd now get published as a Move RPC request on
//      /LocoApiTopicReq (booster_msgs/msg/RpcReqMsg) - same mechanism
//      single_step_node.cpp uses, since this robot has no /cmd_vel
//      interface. Confirmed from the SDK headers: LocoApiId::kMove = 2001,
//      body shape {"vx":..,"vy":..,"vyaw":..}, header shape
//      {"api_id":..,"expect_response":..}.
//      Actually publishing is gated behind the `send` parameter (default
//      false) - same dry-run-by-default pattern as single_step. With
//      send:=false, it only logs what it would publish.
//
// One value the doc does NOT specify: k_heading, the gain in the heading-
// protection branch (w_cmd = k_heading * heading_err). 1.0 here is a
// starting guess, not a confirmed spec value - needs real tuning.
//
// Startup() sends ChangeMode(kWalking) once before the control loop
// starts, since Move commands only take effect once the robot is already
// in walking mode (confirmed via single_step.cpp). Success is confirmed by
// polling /robot_states' current_mode field until it reads back 2
// (kWalking) - not by waiting for a response on /LocoApiTopicResp, which
// was tried first but timed out even when the mode change visibly worked;
// checked live with `ros2 topic info /LocoApiTopicResp --verbose` and
// found Publisher count: 0 - nothing on this robot actually publishes
// responses on that topic.
//

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <booster_msgs/msg/rpc_req_msg.hpp>
#include <booster_interface/msg/robot_states_msg.hpp>

using namespace std::chrono_literals;

namespace {

constexpr int64_t kLocoApiIdChangeMode = 2000;
constexpr int64_t kLocoApiIdMove = 2001;
constexpr int kRobotModeWalking = 2;

double YawFromQuaternion(double w, double x, double y, double z) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

struct Waypoint {
    double x;
    double y;
    double yaw = 0.0;  // radians; only meaningful if the CSV had a yaw column
};

// Reads a waypoints CSV using its header row to find the x/y/yaw columns by
// name, so it works whether given extract_waypoints.py's simple
// (x,y,theta) format or extract_odometry_map.py's richer
// (wp_id,sample_idx,type,t_ns,x,y,z,yaw_rad,cumdist_m) format - both were
// used at different points in this project. has_yaw_out comes back false if
// neither "yaw_rad" nor "theta" is present - callers that need yaw for
// rotation alignment must check that first.
std::vector<Waypoint> LoadWaypoints(const std::string &path, bool &has_yaw_out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open waypoints CSV: " + path);
    }

    std::string header_line;
    if (!std::getline(file, header_line)) {
        throw std::runtime_error("Waypoints CSV is empty: " + path);
    }
    std::vector<std::string> columns;
    std::stringstream header_ss(header_line);
    std::string col;
    while (std::getline(header_ss, col, ',')) columns.push_back(col);

    int x_idx = -1, y_idx = -1, yaw_idx = -1;
    for (size_t i = 0; i < columns.size(); ++i) {
        if (columns[i] == "x") x_idx = static_cast<int>(i);
        if (columns[i] == "y") y_idx = static_cast<int>(i);
        if (columns[i] == "yaw_rad" || columns[i] == "theta") yaw_idx = static_cast<int>(i);
    }
    if (x_idx < 0 || y_idx < 0) {
        throw std::runtime_error("Waypoints CSV header has no x/y columns: " + path);
    }
    has_yaw_out = (yaw_idx >= 0);

    std::vector<Waypoint> waypoints;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::vector<std::string> fields;
        std::stringstream line_ss(line);
        std::string field;
        while (std::getline(line_ss, field, ',')) fields.push_back(field);
        int max_needed = std::max(x_idx, y_idx);
        if (has_yaw_out) max_needed = std::max(max_needed, yaw_idx);
        if (static_cast<int>(fields.size()) <= max_needed) continue;
        Waypoint wp;
        wp.x = std::stod(fields[x_idx]);
        wp.y = std::stod(fields[y_idx]);
        wp.yaw = has_yaw_out ? std::stod(fields[yaw_idx]) : 0.0;
        waypoints.push_back(wp);
    }
    if (waypoints.empty()) {
        throw std::runtime_error("Waypoints CSV loaded but has zero usable rows: " + path);
    }
    return waypoints;
}

std::string GenUuid() {
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist;
    std::ostringstream oss;
    oss << std::hex << dist(gen) << "-" << dist(gen);
    return oss.str();
}

// Confirmed from booster/robot/rpc/request_header.hpp RequestHeader::ToJson().
std::string MakeHeader(int64_t api_id, bool expect_response = false) {
    std::ostringstream oss;
    oss << "{\"api_id\":" << api_id
        << ",\"expect_response\":" << (expect_response ? "true" : "false") << "}";
    return oss.str();
}

// Confirmed from booster/robot/b1/b1_loco_api.hpp MoveParameter::ToJson().
std::string MakeMoveBody(double vx, double vy, double vyaw) {
    std::ostringstream oss;
    oss << "{\"vx\":" << vx << ",\"vy\":" << vy << ",\"vyaw\":" << vyaw << "}";
    return oss.str();
}

// Confirmed from booster/robot/b1/b1_loco_api.hpp ChangeModeParameter::ToJson().
std::string MakeChangeModeBody(int mode) {
    std::ostringstream oss;
    oss << "{\"mode\":" << mode << "}";
    return oss.str();
}

}  // namespace

class PathFollower : public rclcpp::Node {
public:
    PathFollower() : rclcpp::Node("path_follower") {
        declare_parameter<std::string>("waypoints_csv", "");
        declare_parameter<double>("lookahead_m", 0.3);  // matches waypoint spacing (0.3m)
        declare_parameter<double>("v_nom", 0.25);        // doc section 5 default nominal speed
        declare_parameter<double>("v_max", 0.35);
        declare_parameter<double>("w_max", 0.9);
        declare_parameter<double>("v_turn", 0.075);       // mid of doc's 0.05-0.10 range
        declare_parameter<double>("heading_thresh_deg", 35.0);
        declare_parameter<double>("k_heading", 1.0);      // not in doc - needs tuning
        declare_parameter<double>("goal_tol", 0.20);
        declare_parameter<double>("control_hz", 20.0);           // doc section 5: 20-30Hz
        declare_parameter<double>("odom_stale_timeout_s", 0.5);  // Rule 5
        declare_parameter<bool>("send", false);                  // dry-run by default

        std::string csv_path = get_parameter("waypoints_csv").as_string();
        if (csv_path.empty()) {
            throw std::runtime_error(
                "waypoints_csv parameter is required, e.g. --ros-args -p "
                "waypoints_csv:=/home/booster/Workspace/TEST_001_map/waypoints.csv");
        }
        waypoints_ = LoadWaypoints(csv_path, has_yaw_);
        if (!has_yaw_) {
            RCLCPP_WARN(get_logger(), "[CALIBRATE] Waypoints CSV has no yaw column - calibration "
                                       "will only shift position, not rotate to match current heading.");
        }
        lookahead_m_ = get_parameter("lookahead_m").as_double();
        v_nom_ = get_parameter("v_nom").as_double();
        v_max_ = get_parameter("v_max").as_double();
        w_max_ = get_parameter("w_max").as_double();
        v_turn_ = get_parameter("v_turn").as_double();
        heading_thresh_rad_ = get_parameter("heading_thresh_deg").as_double() * M_PI / 180.0;
        k_heading_ = get_parameter("k_heading").as_double();
        goal_tol_ = get_parameter("goal_tol").as_double();
        odom_stale_timeout_s_ = get_parameter("odom_stale_timeout_s").as_double();
        send_ = get_parameter("send").as_bool();

        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s (lookahead_m=%.2f, v_nom=%.2f, send=%s)",
                    waypoints_.size(), csv_path.c_str(), lookahead_m_, v_nom_, send_ ? "true" : "false");
        if (!send_) {
            RCLCPP_WARN(get_logger(), "[DRY RUN] send:=false - will only log what would be published. "
                                       "Pass --ros-args -p send:=true to actually move the robot.");
        }

        move_publisher_ = create_publisher<booster_msgs::msg::RpcReqMsg>(
            "/LocoApiTopicReq", rclcpp::QoS(10).reliable());
        // /LocoApiTopicResp has no confirmed publisher on this robot
        // (checked live via `ros2 topic info /LocoApiTopicResp --verbose`,
        // Publisher count: 0) - so ChangeMode success is confirmed via
        // /robot_states' current_mode field instead, in Startup() below.
        robot_states_sub_ = create_subscription<booster_interface::msg::RobotStatesMsg>(
            "/robot_states", rclcpp::QoS(10).reliable(),
            [this](const booster_interface::msg::RobotStatesMsg::SharedPtr msg) {
                current_mode_ = msg->current_mode;
                got_robot_states_ = true;
            });

        // /odom on this robot publishes best_effort - a default reliable
        // subscriber receives nothing (confirmed earlier in this project).
        // This callback just caches the latest reading now - the timer
        // below does the actual work, at a fixed rate independent of how
        // fast /odom happens to be publishing.
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                last_x_ = msg->pose.pose.position.x;
                last_y_ = msg->pose.pose.position.y;
                last_yaw_ = YawFromQuaternion(
                    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
                have_odom_ = true;
                last_odom_wall_time_ = now();
            });

        control_hz_ = get_parameter("control_hz").as_double();
        // Note: the control-loop timer is NOT created here. It's created
        // at the end of Startup(), below - only after ChangeMode(kWalking)
        // has been confirmed to succeed (or skipped, in dry-run mode). If
        // it were created here, it would start firing ControlCycle() (and
        // therefore publishing Move commands) while Startup() is still
        // spinning the executor to wait for the ChangeMode response -
        // before we know walking mode was actually entered.
    }

    // Waits 2 seconds, spinning so /odom callbacks actually run during
    // that time, letting a few real readings come in and settle before
    // anything below trusts have_odom_/last_x_/last_y_/last_yaw_ - instead
    // of using whatever the very first /odom sample happens to be.
    void WaitForOdomToStabilize() {
        auto deadline = std::chrono::steady_clock::now() + 2000ms;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            rclcpp::spin_some(get_node_base_interface());
            std::this_thread::sleep_for(20ms);
        }
        RCLCPP_INFO(get_logger(), "[ODOM] after 2s wait: have_odom_=%s x=%.3f y=%.3f yaw_deg=%.1f",
                    have_odom_ ? "true" : "false", last_x_, last_y_, last_yaw_ * 180.0 / M_PI);
    }

    // /odom has no fixed absolute origin - it's a local frame that starts
    // at (0,0,0) wherever the robot happened to be when the odometry
    // driver last started up. Confirmed by direct observation: running
    // this same node minutes apart reported wildly different starting
    // positions (x=6.056, then x=0.000, then x=6.293) for what was
    // physically nearly the same spot each time. So a waypoint CSV
    // recorded in a past /odom session is meaningless against today's
    // /odom numbers unless re-anchored first.
    //
    // This assumes the robot is currently standing close to physically
    // where it was when waypoint 0 ("start") was recorded - if that's not
    // true, this calibration will silently produce a wrong, not just
    // stale, path. Relies on WaitForOdomToStabilize() having already run
    // and populated have_odom_/last_x_/last_y_ - doesn't wait on its own.
    bool CalibrateWaypointsToCurrentOdom() {
        if (!have_odom_) {
            RCLCPP_ERROR(get_logger(),
                         "[FAIL] No /odom received - cannot calibrate waypoints to current "
                         "position. Aborting.");
            return false;
        }

        // Same idea as the position offset below (current - recorded), but
        // applied as a rotation of every waypoint's (x,y) around the start
        // point - the steering logic only looks at waypoint positions, so
        // realigning direction means rotating those positions, not just
        // adjusting a separate yaw number.
        double yaw_offset = has_yaw_ ? (last_yaw_ - waypoints_.front().yaw) : 0.0;
        double cos_o = std::cos(yaw_offset);
        double sin_o = std::sin(yaw_offset);

        double orig_start_x = waypoints_.front().x;
        double orig_start_y = waypoints_.front().y;

        for (auto &wp : waypoints_) {
            double dx = wp.x - orig_start_x;
            double dy = wp.y - orig_start_y;
            double rx = cos_o * dx - sin_o * dy;
            double ry = sin_o * dx + cos_o * dy;
            wp.x = last_x_ + rx;
            wp.y = last_y_ + ry;
            wp.yaw += yaw_offset;
        }

        RCLCPP_INFO(get_logger(),
                    "[CALIBRATED] current /odom=(%.3f,%.3f,yaw_deg=%.1f), CSV waypoint 0 was "
                    "(%.3f,%.3f,yaw_deg=%.1f) -> rotated by %.1fdeg and shifted all %zu "
                    "waypoints. New waypoint 0=(%.3f,%.3f), new final=(%.3f,%.3f).%s",
                    last_x_, last_y_, last_yaw_ * 180.0 / M_PI, orig_start_x, orig_start_y,
                    (has_yaw_ ? waypoints_.front().yaw - yaw_offset : 0.0) * 180.0 / M_PI,
                    yaw_offset * 180.0 / M_PI, waypoints_.size(), waypoints_.front().x,
                    waypoints_.front().y, waypoints_.back().x, waypoints_.back().y,
                    has_yaw_ ? "" : " (no yaw column found - rotation is 0deg, position-only)");
        return true;
    }

    // Sends ChangeMode(kWalking) once and confirms it actually took effect
    // by polling /robot_states' current_mode, before the control loop is
    // allowed to start (unlike Move, which is fire-and-forget). Returns
    // false (and does not start the control loop) if send_ is on and this
    // fails. Must be called after construction, before rclcpp::spin().
    bool Startup() {
        WaitForOdomToStabilize();

        if (!CalibrateWaypointsToCurrentOdom()) {
            return false;
        }

        if (!send_) {
            RCLCPP_INFO(get_logger(), "[WOULD SEND] ChangeMode(kWalking) body=%s",
                        MakeChangeModeBody(kRobotModeWalking).c_str());
            StartControlTimer();
            return true;
        }

        // Give discovery a moment so the publisher/subscriber actually
        // match up with the robot's daemon before the first request goes out.
        std::this_thread::sleep_for(1s);

        booster_msgs::msg::RpcReqMsg req;
        req.uuid = GenUuid();
        req.header = MakeHeader(kLocoApiIdChangeMode);
        req.body = MakeChangeModeBody(kRobotModeWalking);

        RCLCPP_INFO(get_logger(), "[SENT] ChangeMode(kWalking) uuid=%s body=%s",
                    req.uuid.c_str(), req.body.c_str());
        move_publisher_->publish(req);

        auto deadline = std::chrono::steady_clock::now() + 5000ms;
        while (rclcpp::ok() && current_mode_ != kRobotModeWalking &&
               std::chrono::steady_clock::now() < deadline) {
            rclcpp::spin_some(get_node_base_interface());
            std::this_thread::sleep_for(20ms);
        }

        if (current_mode_ != kRobotModeWalking) {
            RCLCPP_ERROR(get_logger(),
                         "[FAIL] /robot_states current_mode never reached kWalking(2) within "
                         "5000ms (last seen: %d, got_robot_states=%s). Aborting - control loop "
                         "will not start.",
                         current_mode_, got_robot_states_ ? "true" : "false");
            return false;
        }

        RCLCPP_INFO(get_logger(), "[OK] /robot_states confirms current_mode=kWalking(2).");
        StartControlTimer();
        return true;
    }

private:
    void StartControlTimer() {
        auto period = std::chrono::duration<double>(1.0 / control_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PathFollower::ControlCycle, this));
    }

    void PublishOrLog(double v_cmd, double w_cmd, const char *reason) {
        std::string body = MakeMoveBody(v_cmd, 0.0, w_cmd);
        if (send_) {
            booster_msgs::msg::RpcReqMsg req;
            req.uuid = GenUuid();
            req.header = MakeHeader(kLocoApiIdMove);
            req.body = body;
            RCLCPP_INFO(get_logger(), "[SENT: %s] uuid=%s topic=/LocoApiTopicReq header=%s body=%s",
                        reason, req.uuid.c_str(), req.header.c_str(), body.c_str());
            move_publisher_->publish(req);
        } else {
            RCLCPP_INFO(get_logger(), "[WOULD SEND: %s] Move body=%s", reason, body.c_str());
        }
    }

    void ControlCycle() {
        // Rule 5: safety fallback - no odom yet, or odom gone stale. Only
        // actually publishes a stop if send_ is on and we'd previously be
        // sending real commands; otherwise just logs.
        if (!have_odom_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[STOP] no /odom received yet");
            return;
        }
        double odom_age_s = (now() - last_odom_wall_time_).seconds();
        if (odom_age_s > odom_stale_timeout_s_) {
            RCLCPP_WARN(get_logger(), "[STOP] /odom stale (%.2fs old)", odom_age_s);
            PublishOrLog(0.0, 0.0, "odom stale");
            return;
        }

        const double x = last_x_;
        const double y = last_y_;
        const double yaw = last_yaw_;

        // Waypoint block: forward-only lookahead search (Rule 1).
        while (idx_last_ < waypoints_.size() - 1) {
            double d = std::hypot(waypoints_[idx_last_].x - x, waypoints_[idx_last_].y - y);
            if (d >= lookahead_m_) break;
            ++idx_last_;
        }

        // Steering block.
        double dx = waypoints_[idx_last_].x - x;
        double dy = waypoints_[idx_last_].y - y;
        double y_r = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        double kappa = 2.0 * y_r / (lookahead_m_ * lookahead_m_);
        double v_cmd = v_nom_;
        double w_cmd = v_cmd * kappa;

        // Safety block, Rule 2: heading protection.
        double heading_err = std::atan2(dy, dx) - yaw;
        while (heading_err > M_PI) heading_err -= 2.0 * M_PI;
        while (heading_err < -M_PI) heading_err += 2.0 * M_PI;
        bool heading_protect = std::abs(heading_err) > heading_thresh_rad_;
        if (heading_protect) {
            v_cmd = v_turn_;
            w_cmd = k_heading_ * heading_err;
        }

        // Rule 3: saturate to safe limits.
        v_cmd = std::clamp(v_cmd, 0.0, v_max_);
        w_cmd = std::clamp(w_cmd, -w_max_, w_max_);

        // Rule 4: goal stop - close enough to the final waypoint.
        double d_goal = std::hypot(waypoints_.back().x - x, waypoints_.back().y - y);
        bool at_goal = d_goal < goal_tol_;
        if (at_goal) {
            v_cmd = 0.0;
            w_cmd = 0.0;
        }

        RCLCPP_INFO(get_logger(),
                    "x=%.3f y=%.3f yaw_deg=%.1f | target_idx=%zu/%zu target=(%.3f,%.3f) | "
                    "heading_err_deg=%.1f%s d_goal=%.2f%s | v_cmd=%.3f w_cmd=%.3f",
                    x, y, yaw * 180.0 / M_PI, idx_last_, waypoints_.size() - 1,
                    waypoints_[idx_last_].x, waypoints_[idx_last_].y,
                    heading_err * 180.0 / M_PI, heading_protect ? " [HEADING-PROTECT]" : "",
                    d_goal, at_goal ? " [AT GOAL]" : "", v_cmd, w_cmd);

        PublishOrLog(v_cmd, w_cmd, at_goal ? "at goal" : "normal");

        // Once at the goal, this cycle's zero-velocity command above is
        // the last thing that needs to happen - stop the timer so
        // ControlCycle() doesn't keep running (and logging/publishing
        // increasingly meaningless numbers, since the robot's own
        // momentum keeps drifting position/yaw a little after the stop
        // command, which was swinging heading_err/d_goal around wildly
        // with no motion command actually reacting to them anymore).
        if (at_goal) {
            RCLCPP_INFO(get_logger(), "[DONE] Reached goal, stopping control loop.");
            timer_->cancel();
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<booster_interface::msg::RobotStatesMsg>::SharedPtr robot_states_sub_;
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr move_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> waypoints_;
    bool has_yaw_ = false;
    size_t idx_last_ = 0;
    double lookahead_m_ = 0.3;
    double v_nom_ = 0.25;
    double v_max_ = 0.35;
    double w_max_ = 0.9;
    double v_turn_ = 0.075;
    double heading_thresh_rad_ = 35.0 * M_PI / 180.0;
    double k_heading_ = 1.0;
    double goal_tol_ = 0.20;
    double odom_stale_timeout_s_ = 0.5;
    double control_hz_ = 20.0;
    bool send_ = false;

    double last_x_ = 0.0, last_y_ = 0.0, last_yaw_ = 0.0;
    bool have_odom_ = false;
    rclcpp::Time last_odom_wall_time_;

    // Used only during Startup()'s ChangeMode confirmation, not the
    // continuous fire-and-forget Move commands.
    int32_t current_mode_ = -1;
    bool got_robot_states_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<PathFollower>();
        if (!node->Startup()) {
            rclcpp::shutdown();
            return 1;
        }
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("path_follower"), "[FATAL] %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
