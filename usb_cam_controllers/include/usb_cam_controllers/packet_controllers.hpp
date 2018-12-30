#ifndef USB_CAM_CONTROLLERS_PACKET_CONTROLLERS
#define USB_CAM_CONTROLLERS_PACKET_CONTROLLERS

#include <cmath>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <usb_cam_controllers/simple_packet_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <opencv2/core/core.hpp>

namespace usb_cam_controllers {

class PacketController : public SimplePacketController {
public:
  PacketController() {}

  virtual ~PacketController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    width_ = controller_nh.param("image_width", 640);
    height_ = controller_nh.param("image_height", 480);
    encoding_ = controller_nh.param< std::string >("encoding", sensor_msgs::image_encodings::BGR8);
    skip_max_ = std::max(controller_nh.param("skip", /* pub every packet */ 0), 0);

    skip_cnt_ = skip_max_;
    publisher_ = image_transport::ImageTransport(controller_nh).advertise("image", 1);

    return true;
  }

  virtual void startingImpl(const ros::Time &time) {
    // nothig to do
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // publish the current packet if enough number of previous packets are skipped
    if (skip_cnt_ >= skip_max_) {
      // reset skip count
      skip_cnt_ = 0;

      // publish the packet
      cv_bridge::CvImage out;
      out.header.stamp = packet_iface_.getStamp();
      out.encoding = encoding_;
      out.image = cv::Mat(height_, width_, CV_8UC(packet_iface_.getLength() / (height_ * width_)),
                          const_cast< uint8_t * >(packet_iface_.getStartAs< uint8_t >()));
      publisher_.publish(out.toImageMsg());
    } else {
      // increment skip count if the current packet is skipped
      ++skip_cnt_;
    }
  }

  virtual void stoppingImpl(const ros::Time &time) {
    // nothing to do
  }

private:
  std::string encoding_;
  int height_, width_;
  int skip_max_;

  int skip_cnt_;
  image_transport::Publisher publisher_;
};

class CompressedPacketController : public SimplePacketController {
public:
  CompressedPacketController() {}

  virtual ~CompressedPacketController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    format_ = controller_nh.param< std::string >("format", "jpeg");
    skip_max_ = std::max(controller_nh.param("skip", /* pub every packet */ 0), 0);

    skip_cnt_ = skip_max_;
    publisher_ = controller_nh.advertise< sensor_msgs::CompressedImage >("packet", 1);

    return true;
  }

  virtual void startingImpl(const ros::Time &time) {
    // nothig to do
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // publish the current packet if enough number of previous packets are skipped
    if (skip_cnt_ >= skip_max_) {
      // reset skip count
      skip_cnt_ = 0;

      // publish the packet
      const sensor_msgs::CompressedImagePtr msg(new sensor_msgs::CompressedImage());
      msg->header.stamp = packet_iface_.getStamp();
      msg->format = format_;
      msg->data.assign(packet_iface_.getStartAs< uint8_t >(),
                       packet_iface_.getStartAs< uint8_t >() + packet_iface_.getLength());
      publisher_.publish(msg);
    } else {
      // increment skip count if the current packet is skipped
      ++skip_cnt_;
    }
  }

  virtual void stoppingImpl(const ros::Time &time) {
    // nothing to do
  }

private:
  std::string format_;
  int skip_max_;

  int skip_cnt_;
  ros::Publisher publisher_;
};

} // namespace usb_cam_controllers

#endif