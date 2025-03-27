#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <libav_cpp/libav_cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace av = libav_cpp;

int main(int argc, char *argv[]) {
  // Initialize ROS 2 node
  rclcpp::init(argc, argv);
  const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("decode_topic");

  // Receive one message from the /image/compressed topic to determine the decoder name
  RCLCPP_INFO(node->get_logger(), "Waiting for the first message to determine the decoder name");
  const std::string decoder_name = [&node]() {
    std::promise<std::string> promise;
    const auto subscription = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", 1,
        [&promise](const sensor_msgs::msg::CompressedImage::SharedPtr packet_data) {
          if (!packet_data->format.empty()) {
            promise.set_value(packet_data->format);
          }
        });
    auto future = promise.get_future();
    rclcpp::spin_until_future_complete(node, future);
    return future.get();
  }();
  RCLCPP_INFO(node->get_logger(), "Received the first message with the decoder name: %s",
              decoder_name.c_str());

  try {
    // Initialize the decoder, which decompresses H.264 video packets
    // and the converter, which converts the decoded frames to BGR24 images
    av::Packet packet;
    av::Frame frame, sw_frame;
    av::Decoder decoder(decoder_name);
    av::Converter converter("bgr24");
    RCLCPP_INFO(node->get_logger(),
                "Initialized decoder (codec: %s, hw: %s) and converter (dst: %s)",
                decoder.codec_name().c_str(), decoder.hw_device_type().c_str(),
                converter.dst_format_name().c_str());

    // Create a publisher to publish the decoded images
    const auto publisher = node->create_publisher<sensor_msgs::msg::Image>("dst_image", 10);

    // Define the callback function to decode and publish the H.264 packets
    const auto decode_and_publish =
        [&](const sensor_msgs::msg::CompressedImage::SharedPtr packet_data) {
          try {
            // Set the packet data from the message
            if (packet_data->data.empty()) {
              return;
            }
            packet->data = packet_data->data.data();
            packet->size = packet_data->data.size();

            // Send the packet to the decoder
            decoder.send_packet(packet);

            // Receive and publish the decoded frames
            while (decoder.receive_frame(&frame)) {
              // Copy the frame properties to the destination image
              auto image = std::make_unique<sensor_msgs::msg::Image>();
              image->header.stamp = packet_data->header.stamp;
              image->height = frame->height;
              image->width = frame->width;
              image->encoding = sensor_msgs::image_encodings::BGR8;
              image->step = 3 * frame->width;

              // Convert the frame to BGR24 image
              if (frame.is_hw_frame()) {
                // If the frame data is in a hardware device,
                // transfer the data to the CPU-accessible memory before conversion
                frame.transfer_data(&sw_frame);
                converter.convert(sw_frame, &image->data);
              } else {
                converter.convert(frame, &image->data);
              }

              // Publish the destination image
              publisher->publish(std::move(image));
            }
          } catch (const std::runtime_error &err) {
            RCLCPP_ERROR(node->get_logger(), "Error: %s", err.what());
          }
        };

    const auto subscription = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", 10, decode_and_publish);

    rclcpp::spin(node);
  } catch (const std::runtime_error &err) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", err.what());
  }

  rclcpp::shutdown();

  return 0;
}
