#include <memory>
#include <stdexcept>
#include <string>

#include <libav_cpp/libav_cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace av = libav_cpp;

// Utility function to deserialize a message from a bag
template <class Msg>
std::unique_ptr<Msg> deserialize(rclcpp::Serialization<Msg> *const deserializer,
                                 const rosbag2_storage::SerializedBagMessage &bag_msg) {
  const rclcpp::SerializedMessage serialized_msg(*bag_msg.serialized_data);
  auto deserialized_msg = std::make_unique<Msg>();
  deserializer->deserialize_message(&serialized_msg, deserialized_msg.get());
  return deserialized_msg;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s <bag_file>\n", argv[0]);
    return -1;
  }
  const std::string bag_filename = argv[1];

  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("bag_decoder");
  const auto publisher = node->create_publisher<sensor_msgs::msg::Image>("dst_image", 10);

  // Initialize the bag reader with the specified file
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_filename;
  const auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  reader->open(storage_options);

  // Initialize the deserializer for messages in the bag
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;

  // Read the first message to get the decoder name
  std::string decoder_name;
  while (decoder_name.empty() && reader->has_next() && rclcpp::ok()) {
    const auto bag_msg = reader->read_next();
    if (bag_msg->topic_name == "/image/compressed") {
      const auto comp_img = deserialize(&serialization, *bag_msg);
      if (!comp_img->format.empty()) {
        decoder_name = comp_img->format;
      }
    }
  }
  RCLCPP_INFO(node->get_logger(), "Read the first message with the decoder name: %s",
              decoder_name.c_str());

  // Initialize the decoder and converter
  av::Parser parser(decoder_name);
  av::Decoder decoder(decoder_name);
  av::Converter converter("bgr24");
  RCLCPP_INFO(node->get_logger(), "Initialized decoder (codec: %s, hw: %s) and converter (dst: %s)",
              decoder.codec_name().c_str(), decoder.hw_device_type().c_str(),
              converter.dst_format_name().c_str());

  // Decode and convert each message in the bag
  const rclcpp::Time start_time = node->now();
  std::size_t n_decoded = 0, n_error = 0;
  while (reader->has_next() && rclcpp::ok()) {
    const auto bag_msg = reader->read_next();
    if (bag_msg->topic_name != "/image/compressed") {
      continue;
    }

    const auto comp_img = deserialize(&serialization, *bag_msg);

    try {
      // Copy the compressed data to the reference-counted buffer with padding
      const av::BufferRef buffer(comp_img->data.data(), comp_img->data.size());

      // Parse the buffer and decode the compressed data
      for (std::size_t pos = 0; pos < buffer.unpadded_size();) {
        // Parse the buffer from the current position and store the data in the packet
        av::Packet packet;
        pos += parser.parse(buffer, &decoder, &packet, pos);

        if (packet->data) {
          // Send the packet to the decoder
          decoder.send_packet(packet);
          RCLCPP_INFO(node->get_logger(), "Sent packet to the decoder: %d bytes", packet->size);

          // Receive and publish the decoded frames
          av::Frame frame;
          while (decoder.receive_frame(&frame)) {
            RCLCPP_INFO(node->get_logger(), "Decoded frame: %dx%d", frame->width, frame->height);
            // Copy the frame properties to the destination image
            auto image = std::make_unique<sensor_msgs::msg::Image>();
            image->header.stamp = comp_img->header.stamp;
            image->height = frame->height;
            image->width = frame->width;
            image->encoding = sensor_msgs::image_encodings::BGR8;
            image->step = 3 * frame->width;

            // Convert the frame to BGR24 image
            if (frame.is_hw_frame()) {
              // If the frame data is in a hardware device,
              // transfer the data to the CPU-accessible memory before conversion
              av::Frame sw_frame;
              frame.transfer_data(&sw_frame);
              converter.convert(sw_frame, &image->data);
            } else {
              converter.convert(frame, &image->data);
            }

            // Publish the destination image
            publisher->publish(std::move(image));
            ++n_decoded;
          }
        }
      }
    } catch (const std::runtime_error &error) {
      ++n_error;
      RCLCPP_ERROR(node->get_logger(), "Error while decoding: %s, will try the next message",
                   error.what());
      continue;
    }
  }

  // Print the decoding statistics
  const rclcpp::Duration elapsed_time = node->now() - start_time;
  RCLCPP_INFO(node->get_logger(), "Decoded %zu frames, %zu errors, in %f seconds (%f fps)",
              n_decoded, n_error, elapsed_time.seconds(),
              (n_decoded + n_error) / elapsed_time.seconds());

  rclcpp::shutdown();

  return 0;
}
