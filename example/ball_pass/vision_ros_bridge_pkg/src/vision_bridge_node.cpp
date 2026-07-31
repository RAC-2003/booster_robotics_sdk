/**
 * vision_bridge_node.cpp
 *
 * Standalone process: subscribes to robocup_demo's vision topic
 * (/booster_soccer/detection) and writes the latest detections to a tmpfs
 * file that a completely separate, non-ROS2 process can read.
 *
 * Why a separate PROCESS (not just a separate library/translation unit):
 * ROS2 Humble's rmw_fastrtps_cpp is built against libfastrtps.so.2.6, while
 * the Booster SDK (ChannelFactory/B1LocoClient) links libfastrtps.so.2.13 -
 * two genuinely different, ABI-incompatible versions of the same library.
 * Confirmed by crash: creating an rclcpp::Node in the same process that also
 * has the SDK's FastDDS loaded throws std::bad_array_new_length deep inside
 * rmw_fastrtps_shared_cpp::create_participant, from corrupted struct layout.
 * Keeping ROS2 in its own process means only one FastDDS version is ever
 * loaded per process - no ABI collision possible.
 *
 * File format (plain text, no JSON dependency needed on the reading side):
 *   line 1: "<write_ts_ms> <capture_ts_ms>" - write_ts_ms is when this
 *           process wrote the file (staleness check); capture_ts_ms is the
 *           original camera frame's own timestamp (msg->header.stamp,
 *           forwarded by vision_node from the source image), letting a
 *           reader measure true end-to-end pipeline latency
 *           (now - capture_ts_ms), not just the last-leg bridge-to-reader
 *           gap. 0 if the incoming message's header stamp was unset.
 *   line 2+: one detection per line, "label|confidence|x,y,z"
 *            (position field is empty string if no position estimate)
 * Written to a temp file then renamed over the real path, so the reader
 * never sees a partially-written file (rename() is atomic on POSIX).
 *
 * Optional pose/calibration recording (off by default, doesn't affect
 * ball_chase.cpp's normal use): when the "pose_output_basename" ROS2
 * parameter is set to a non-empty path prefix, this node also subscribes
 * to /booster_soccer/t_head2base (p_head2base - vision_node's own live
 * head pose, republished verbatim as a TF) and /booster_soccer/cal_param
 * (the pitch/yaw/z compensation inputs to p_headprime2head_ - see
 * vision_node.cpp's CalParamCallback for the exact formula it builds from
 * these), appending a timestamped CSV row to <basename>_pose.csv and
 * <basename>_calparam.csv respectively on every message received. This is
 * two independently-arriving streams, not synced/merged into one file -
 * same reasoning as sensor_log.cpp's separate IMU/motor CSVs.
 *
 * p_eye2head (the third pose in vision_node's p_eye2base chain) is not
 * recorded here at all - it's a fixed value loaded once from vision.yaml
 * and never changes at runtime, so there's nothing to subscribe to. Find
 * it directly in the deployed config's camera.extrin block:
 *   /home/booster/Workspace/build_no_ros/vision_config/vision.yaml
 */
#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vision_interface/msg/cal_param.hpp>
#include <vision_interface/msg/detections.hpp>

namespace {

constexpr const char* kOutputPath = "/dev/shm/booster_vision_bridge.txt";
constexpr const char* kOutputTmpPath = "/dev/shm/booster_vision_bridge.txt.tmp";

long long NowEpochMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

class VisionBridgeNode : public rclcpp::Node {
public:
    VisionBridgeNode() : Node("vision_bridge_node") {
        subscription_ = this->create_subscription<vision_interface::msg::Detections>(
            "/booster_soccer/detection", rclcpp::QoS(1),
            [this](const vision_interface::msg::Detections::SharedPtr msg) {
                WriteDetections(msg);
            });
        RCLCPP_INFO(this->get_logger(),
            "vision_bridge_node: subscribed to /booster_soccer/detection, "
            "writing to %s", kOutputPath);

        this->declare_parameter<std::string>("pose_output_basename", "");
        const std::string pose_basename = this->get_parameter("pose_output_basename").as_string();
        if (!pose_basename.empty()) {
            pose_csv_.open(pose_basename + "_pose.csv");
            pose_csv_ << "epoch_ms,tx,ty,tz,qx,qy,qz,qw\n";
            calparam_csv_.open(pose_basename + "_calparam.csv");
            calparam_csv_ << "epoch_ms,pitch_compensation,yaw_compensation,z_compensation\n";

            pose_subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
                "/booster_soccer/t_head2base", rclcpp::QoS(10),
                [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
                    WritePose(msg);
                });
            calparam_subscription_ = this->create_subscription<vision_interface::msg::CalParam>(
                "/booster_soccer/cal_param", rclcpp::QoS(10),
                [this](const vision_interface::msg::CalParam::SharedPtr msg) {
                    WriteCalParam(msg);
                });
            RCLCPP_INFO(this->get_logger(),
                "vision_bridge_node: also recording p_head2base + cal_param to %s_pose.csv / %s_calparam.csv",
                pose_basename.c_str(), pose_basename.c_str());
        }
    }

private:
    void WritePose(const geometry_msgs::msg::TransformStamped::SharedPtr& msg) {
        const auto& t = msg->transform.translation;
        const auto& r = msg->transform.rotation;
        pose_csv_ << NowEpochMs() << "," << t.x << "," << t.y << "," << t.z << ","
                  << r.x << "," << r.y << "," << r.z << "," << r.w << "\n";
        pose_csv_.flush();
    }

    void WriteCalParam(const vision_interface::msg::CalParam::SharedPtr& msg) {
        calparam_csv_ << NowEpochMs() << "," << msg->pitch_compensation << "," << msg->yaw_compensation << ","
                      << msg->z_compensation << "\n";
        calparam_csv_.flush();
    }

    void WriteDetections(const vision_interface::msg::Detections::SharedPtr& msg) {
        std::ofstream out(kOutputTmpPath, std::ios::trunc);
        if (!out.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "failed to open %s for writing", kOutputTmpPath);
            return;
        }

        const long long capture_ts_ms =
            static_cast<long long>(msg->header.stamp.sec) * 1000LL +
            static_cast<long long>(msg->header.stamp.nanosec) / 1000000LL;
        out << NowEpochMs() << " " << capture_ts_ms << "\n";
        for (const auto& obj : msg->detected_objects) {
            out << obj.label << "|" << obj.confidence << "|";

            // vision_node only fills `position` when depth-based estimation
            // is enabled (use_depth: true); with it disabled, `position` is
            // always (0,0,0) and the real ground-intersection estimate ends
            // up in `position_projection` instead. Prefer `position` when
            // it's actually non-zero, otherwise fall back to the projection.
            bool position_is_zero = obj.position.empty();
            if (!position_is_zero) {
                position_is_zero = true;
                for (float v : obj.position) {
                    if (v != 0.0f) {
                        position_is_zero = false;
                        break;
                    }
                }
            }
            const auto& pos = position_is_zero ? obj.position_projection : obj.position;

            for (size_t i = 0; i < pos.size(); ++i) {
                if (i > 0) out << ",";
                out << pos[i];
            }
            out << "\n";
        }
        out.close();

        // Atomic swap so the reader (a different process) never sees a
        // half-written file.
        if (std::rename(kOutputTmpPath, kOutputPath) != 0) {
            RCLCPP_ERROR(this->get_logger(), "failed to rename %s -> %s",
                         kOutputTmpPath, kOutputPath);
        }
    }

    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr subscription_;

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<vision_interface::msg::CalParam>::SharedPtr calparam_subscription_;
    std::ofstream pose_csv_;
    std::ofstream calparam_csv_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
