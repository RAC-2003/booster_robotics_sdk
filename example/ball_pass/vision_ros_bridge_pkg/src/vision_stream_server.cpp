/**
 * vision_stream_server.cpp
 *
 * Standalone process: subscribes to robocup_demo's camera image and its
 * detection topic, draws bounding boxes + label + confidence + distance on
 * each frame, and serves the annotated video as an MJPEG stream over plain
 * HTTP - viewable from any browser, including remotely over an SSH port
 * forward (`ssh -L 8090:localhost:8090 <user>@<robot-ip>`, then open
 * http://localhost:8090 locally), with no X11 forwarding or GUI needed on
 * either end.
 *
 * Kept as its own process for the same reason as vision_bridge_node: ROS2
 * Humble's rmw_fastrtps_cpp and the Booster SDK need different, ABI
 * incompatible FastDDS versions, so anything using ROS2 topics has to stay
 * out of process from anything using the SDK's ChannelFactory.
 */
#include <chrono>
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
#include <vision_interface/msg/detections.hpp>

namespace {

constexpr int kHttpPort = 8090;
constexpr double kStreamFps = 10.0;
constexpr int kJpegQuality = 80;
// Detections older than this aren't drawn - stale boxes on a live frame are
// more misleading than an occasional frame with no boxes at all.
constexpr int64_t kDetectionMaxAgeMs = 1000;

float Dist2D(const std::vector<float>& pos) {
    if (pos.size() < 2) return -1.0f;
    return std::hypot(pos[0], pos[1]);
}

long long NowEpochMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

class FrameBuffer {
public:
    void Set(std::vector<uchar> jpeg) {
        std::lock_guard<std::mutex> lock(mutex_);
        jpeg_ = std::move(jpeg);
        ++version_;
    }

    // Returns the current frame and its version. If version matches
    // last_seen_version, the caller already has the latest frame.
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
// parsing - every connection just gets the multipart stream.
void RunHttpServer() {
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
    addr.sin_port = htons(kHttpPort);

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Failed to bind HTTP server to port " << kHttpPort << std::endl;
        close(server_fd);
        return;
    }
    listen(server_fd, 8);
    std::cout << "vision_stream_server: MJPEG stream on http://0.0.0.0:" << kHttpPort << "/" << std::endl;

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
            const auto frame_interval = std::chrono::duration<double>(1.0 / kStreamFps);

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
                }
                std::this_thread::sleep_for(frame_interval);
            }
            close(client_fd);
        }).detach();
    }

    close(server_fd);
}

class VisionStreamServer : public rclcpp::Node {
public:
    VisionStreamServer() : Node("vision_stream_server") {
        this->declare_parameter<std::string>("color_topic", "/boostercamera/head/rgb");
        std::string color_topic = this->get_parameter("color_topic").as_string();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic, rclcpp::QoS(1).best_effort(),
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                OnImage(msg);
            });

        detections_sub_ = this->create_subscription<vision_interface::msg::Detections>(
            "/booster_soccer/detection", rclcpp::QoS(1),
            [this](const vision_interface::msg::Detections::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(detections_mutex_);
                latest_detections_ = msg;
                latest_detections_at_ms_ = NowEpochMs();
            });

        RCLCPP_INFO(this->get_logger(),
            "vision_stream_server: subscribed to %s and /booster_soccer/detection, "
            "streaming annotated frames on http://0.0.0.0:%d/",
            color_topic.c_str(), kHttpPort);
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
                // own YUV->BGR conversion do the actual decode.
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

        vision_interface::msg::Detections::SharedPtr detections;
        int64_t detections_age_ms = -1;
        {
            std::lock_guard<std::mutex> lock(detections_mutex_);
            detections = latest_detections_;
            if (detections) {
                detections_age_ms = NowEpochMs() - latest_detections_at_ms_;
            }
        }

        if (detections && detections_age_ms >= 0 && detections_age_ms <= kDetectionMaxAgeMs) {
            for (const auto& obj : detections->detected_objects) {
                const cv::Point top_left(static_cast<int>(obj.xmin), static_cast<int>(obj.ymin));
                const cv::Point bottom_right(static_cast<int>(obj.xmax), static_cast<int>(obj.ymax));

                const bool is_ball = (obj.label == "Ball" || obj.label == "ball");
                const cv::Scalar color = is_ball ? cv::Scalar(0, 215, 255)   // orange/yellow, BGR
                                                  : cv::Scalar(0, 255, 0);    // green
                cv::rectangle(frame, top_left, bottom_right, color, 2);

                std::vector<float> pos = obj.position;
                bool pos_is_zero = pos.empty();
                if (!pos_is_zero) {
                    pos_is_zero = true;
                    for (float v : pos) {
                        if (v != 0.0f) { pos_is_zero = false; break; }
                    }
                }
                const auto& used_pos = pos_is_zero ? obj.position_projection : pos;
                const float dist = Dist2D(used_pos);

                std::ostringstream label;
                label << obj.label << " " << static_cast<int>(obj.confidence) << "%";
                if (dist >= 0.0f) {
                    char dist_buf[32];
                    std::snprintf(dist_buf, sizeof(dist_buf), " %.2fm", dist);
                    label << dist_buf;
                }

                const int text_y = std::max(0, top_left.y - 8);
                cv::putText(frame, label.str(), cv::Point(top_left.x, text_y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
            }
        }

        std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
        std::vector<uchar> jpeg;
        cv::imencode(".jpg", frame, jpeg, jpeg_params);
        g_frame_buffer.Set(std::move(jpeg));
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr detections_sub_;

    std::mutex detections_mutex_;
    vision_interface::msg::Detections::SharedPtr latest_detections_;
    int64_t latest_detections_at_ms_ = 0;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::thread http_thread(RunHttpServer);

    rclcpp::spin(std::make_shared<VisionStreamServer>());

    g_run = false;
    // The HTTP server's accept() call blocks until a connection or process
    // exit interrupts it - detach rather than join so shutdown isn't stuck
    // waiting on a client that may never come.
    http_thread.detach();

    rclcpp::shutdown();
    return 0;
}
