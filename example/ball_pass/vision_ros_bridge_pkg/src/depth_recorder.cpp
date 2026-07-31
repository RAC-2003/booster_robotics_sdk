/**
 * depth_recorder.cpp
 *
 * Subscribes to the camera driver's stereo-computed depth topic
 * (/boostercamera/head/depth, sensor_msgs::msg::Image) and writes each
 * frame's raw pixel bytes to its own file, losslessly - no video encoding
 * (h264/MJPEG) applied, since that would corrupt the actual per-pixel
 * distance values the way it's acceptable to for a color video.
 *
 * This depth topic comes from the camera driver itself (cam_stream_receiver_ros2,
 * a separate process from vision_node), NOT from vision_node - it exists
 * and carries real data independent of vision_node's own use_depth config
 * flag (false on this robot; irrelevant here).
 *
 * Output: <output_dir>/info.txt, written once from the first frame
 * (width/height/encoding/step), plus one <output_dir>/<epoch_ms>.raw per
 * frame after that, containing exactly msg->data verbatim - no assumption
 * is made here about bytes-per-pixel; info.txt's encoding field (whatever
 * the topic actually reports, e.g. "16UC1"/"32FC1") is what a later
 * reader (e.g. numpy) needs to interpret the raw bytes correctly.
 *
 * Kept as its own process for the same reason as every other node in this
 * package: ROS2 Humble's rmw_fastrtps_cpp and the Booster SDK need
 * different, ABI-incompatible FastDDS versions - see vision_bridge_node.cpp
 * for the full explanation.
 *
 * Usage:
 *   ./depth_recorder <output_dir>
 *   (output_dir is created if it doesn't already exist)
 */
#include <sys/stat.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

long long NowEpochMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

class DepthRecorder : public rclcpp::Node {
public:
    explicit DepthRecorder(std::string output_dir) : Node("depth_recorder"), output_dir_(std::move(output_dir)) {
        mkdir(output_dir_.c_str(), 0755);  // fine if it already exists

        this->declare_parameter<std::string>("depth_topic", "/boostercamera/head/depth");
        const std::string depth_topic = this->get_parameter("depth_topic").as_string();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, rclcpp::QoS(1).best_effort(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { OnImage(msg); });

        RCLCPP_INFO(this->get_logger(), "depth_recorder: subscribed to %s, writing raw frames to %s/",
                    depth_topic.c_str(), output_dir_.c_str());
    }

private:
    void OnImage(const sensor_msgs::msg::Image::SharedPtr &msg) {
        if (!info_written_) {
            std::ofstream info(output_dir_ + "/info.txt");
            info << "width=" << msg->width << "\n"
                 << "height=" << msg->height << "\n"
                 << "encoding=" << msg->encoding << "\n"
                 << "step=" << msg->step << "\n";
            info_written_ = true;
        }

        std::ofstream out(output_dir_ + "/" + std::to_string(NowEpochMs()) + ".raw", std::ios::binary);
        out.write(reinterpret_cast<const char *>(msg->data.data()), static_cast<std::streamsize>(msg->data.size()));
    }

    std::string output_dir_;
    bool info_written_ = false;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <output_dir>\n";
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthRecorder>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
