/**
 * raw_stream_server.cpp
 *
 * Standalone process: subscribes to robocup_demo's raw camera image topic
 * ONLY (no detections) and serves it as an MJPEG stream over plain HTTP,
 * unmodified - no bounding boxes, labels, or any other overlay. For
 * recording ground-truth footage (e.g. running-track lane lines) where the
 * pixels need to be exactly what the camera saw.
 *
 * Frames are re-encoded to JPEG at maximum quality purely to get them off
 * the robot over a socket - no resize/crop/color-grade is applied. This is
 * the same unavoidable transport step record_video.sh's ffmpeg capture
 * already relies on; the JPEG quality here is set higher than
 * vision_stream_server's (which is a live debug view, not a recording
 * source) to minimize compression artifacts on the fine line edges a later
 * PID lane controller would need.
 *
 * Kept as its own process for the same reason as vision_bridge_node /
 * vision_stream_server: ROS2 Humble's rmw_fastrtps_cpp and the Booster SDK
 * need different, ABI-incompatible FastDDS versions, so anything using ROS2
 * topics has to stay out of process from anything using the SDK's
 * ChannelFactory.
 *
 * Runs on a different port (8091) than vision_stream_server (8090) so both
 * can run at the same time without colliding.
 */
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

constexpr int kHttpPort = 8091;
constexpr int kJpegQuality = 100;

class FrameBuffer {
public:
    void Set(std::vector<uchar> jpeg) {
        std::lock_guard<std::mutex> lock(mutex_);
        jpeg_ = std::move(jpeg);
        ++version_;
    }

    std::vector<uchar> Get(uint64_t& version) const {
        std::lock_guard<std::mutex> lock(mutex_);
        version = version_;
        return jpeg_;
    }

private:
    mutable std::mutex mutex_;
    std::vector<uchar> jpeg_;
    uint64_t version_ = 0;
};

FrameBuffer g_frame_buffer;
std::atomic<bool> g_run{true};

// Minimal single-purpose MJPEG-over-HTTP server. No routing, no headers
// parsing - every connection just gets the multipart stream. Pushes every
// new frame as it arrives (no artificial fps cap), unlike
// vision_stream_server, since this is meant to be recorded frame-accurately
// rather than watched live.
//
// Takes the port as a parameter (not the kHttpPort constant directly) so a
// second instance can run alongside the first on a different port - e.g.
// pointed at the right-eye topic instead of left, for stereo recording.
void RunHttpServer(int http_port) {
    const int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "Failed to create HTTP server socket." << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(http_port));

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Failed to bind HTTP server to port " << http_port << std::endl;
        close(server_fd);
        return;
    }
    listen(server_fd, 8);
    std::cout << "raw_stream_server: MJPEG stream on http://0.0.0.0:" << http_port << "/" << std::endl;

    while (g_run.load()) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        const int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            continue;
        }

        std::thread([client_fd]() {
            static const char* kHeader =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n";
            if (send(client_fd, kHeader, std::strlen(kHeader), 0) < 0) {
                close(client_fd);
                return;
            }

            uint64_t last_version = 0;

            while (g_run.load()) {
                uint64_t version = 0;
                std::vector<uchar> jpeg = g_frame_buffer.Get(version);
                if (!jpeg.empty() && version != last_version) {
                    last_version = version;
                    std::ostringstream part_header;
                    part_header << "--frame\r\n"
                                << "Content-Type: image/jpeg\r\n"
                                << "Content-Length: " << jpeg.size() << "\r\n\r\n";
                    const std::string header_str = part_header.str();

                    if (send(client_fd, header_str.data(), header_str.size(), MSG_NOSIGNAL) < 0 ||
                        send(client_fd, jpeg.data(), jpeg.size(), MSG_NOSIGNAL) < 0 ||
                        send(client_fd, "\r\n", 2, MSG_NOSIGNAL) < 0) {
                        break;  // client disconnected
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            }
            close(client_fd);
        }).detach();
    }

    close(server_fd);
}

class RawStreamServer : public rclcpp::Node {
public:
    RawStreamServer() : Node("raw_stream_server") {
        this->declare_parameter<std::string>("color_topic", "/boostercamera/head/rgb");
        std::string color_topic = this->get_parameter("color_topic").as_string();

        this->declare_parameter<int>("http_port", kHttpPort);
        http_port_ = this->get_parameter("http_port").as_int();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic, rclcpp::QoS(1).best_effort(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                OnImage(msg);
            });

        RCLCPP_INFO(this->get_logger(),
            "raw_stream_server: subscribed to %s (no detections, no overlay), "
            "streaming raw frames on http://0.0.0.0:%d/",
            color_topic.c_str(), http_port_);
    }

private:
    void OnImage(const sensor_msgs::msg::Image::SharedPtr& msg) {
        cv::Mat frame;
        try {
            if (msg->encoding == "nv12") {
                // cv_bridge has no built-in NV12 decoder. NV12 is a
                // semi-planar YUV 4:2:0 layout: a full-resolution Y plane
                // followed by a half-resolution interleaved UV plane, so the
                // raw buffer is exactly height*1.5 rows of `width` bytes -
                // build a Mat over that raw layout directly and let OpenCV's
                // own YUV->BGR conversion do the actual decode. This is a
                // colorspace decode, not an alteration of the image content.
                cv::Mat nv12(msg->height + msg->height / 2, msg->width, CV_8UC1,
                             const_cast<uint8_t*>(msg->data.data()));
                cv::cvtColor(nv12, frame, cv::COLOR_YUV2BGR_NV12);
            } else {
                frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "image conversion failed (encoding=%s): %s",
                         msg->encoding.c_str(), e.what());
            return;
        }
        if (frame.empty()) return;

        std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
        std::vector<uchar> jpeg;
        cv::imencode(".jpg", frame, jpeg, jpeg_params);
        g_frame_buffer.Set(std::move(jpeg));
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    int http_port_ = kHttpPort;

public:
    int GetHttpPort() const { return http_port_; }
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Node constructed (and its http_port parameter read) before the HTTP
    // thread starts, since the thread needs that value up front.
    auto node = std::make_shared<RawStreamServer>();
    std::thread http_thread(RunHttpServer, node->GetHttpPort());

    rclcpp::spin(node);

    g_run = false;
    http_thread.detach();

    rclcpp::shutdown();
    return 0;
}
