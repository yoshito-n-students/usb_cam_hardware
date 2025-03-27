#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <libav_cpp/libav_cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace av = libav_cpp;

int main(int argc, char *argv[]) {
  // ROS 2 node initialization
  rclcpp::init(argc, argv);
  const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("stream_camera");

  // Parameters for the input device
  const std::string url = node->declare_parameter("url", "/dev/video0");
  const std::map<std::string, std::string> option_map = {
      {"input_format", node->declare_parameter("input_format", "h264")},
      {"video_size", node->declare_parameter("video_size", "1920x1080")},
      {"framerate", node->declare_parameter("framerate", "30")}};

  // Setup the destination packet publisher
  const auto publisher =
      node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10);

  try {
    // Open the input device
    av::Input input(url, "v4l2", option_map);
    av::Packet packet;

    // Determine the format name for the message based on the codec ID of the input stream
    const std::string msg_format = [&](const AVCodecID codec_id) -> std::string {
      if (codec_id == AV_CODEC_ID_MJPEG) {
        return "jpeg"; // ROS 2 expects "jpeg" instead of "mjpeg"
      } else {
        const std::string codec_name = avcodec_get_name(codec_id);
        RCLCPP_INFO(node->get_logger(),
                    "Using the codec name from libav (%s) as the format name for messages",
                    codec_name.c_str());
        return codec_name;
      }
    }(input->streams[input.stream_id()]->codecpar->codec_id);

    // Continuously read frames from the input device and publish them
    while (rclcpp::ok()) {
      // Read a packet from the input device
      // (TODO: enable timeout or break on !rclcpp::ok())
      input.read_frame(&packet);

      // Copy the packet data to a ROS 2 message
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      // packet->pts is timestamp in kernel time, in microseconds
      msg->header.stamp.sec = packet->pts / 1'000'000;
      msg->header.stamp.nanosec = (packet->pts % 1'000'000) * 1'000;
      msg->format = msg_format;
      msg->data.assign(packet->data, packet->data + packet->size);

      // Publish the message
      publisher->publish(std::move(msg));
    }
  } catch (const std::runtime_error &error) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", error.what());
  }

  rclcpp::shutdown();

  return 0;
}