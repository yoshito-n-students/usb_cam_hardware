#ifndef USB_CAM_CONTROLLERS_PACKET_CONTROLLER
#define USB_CAM_CONTROLLERS_PACKET_CONTROLLER

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <sensor_msgs/CompressedImage.h>
#include <usb_cam_hardware_interface/packet_interface.hpp>

namespace usb_cam_controllers {

class PacketController
    : public controller_interface::Controller< usb_cam_hardware_interface::PacketInterface > {
public:
  PacketController() {}

  virtual bool init(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    format_ = controller_nh.param< std::string >("format", "");

    if (!hw) {
      ROS_ERROR("Null packet interface");
      return false;
    }

    const std::vector< std::string > names(hw->getNames());
    if (names.empty()) {
      ROS_ERROR("No packet handle");
      return false;
    }
    if (names.size() > 1) {
      ROS_WARN_STREAM(names.size()
                      << " packet handles. packets only from the first handle will be published.");
    }

    packet_ = hw->getHandle(names.front());
    publisher_ = controller_nh.advertise< sensor_msgs::CompressedImage >("packet", 1);
    last_stamp_ = ros::Time(0);

    return true;
  }

  virtual void starting(const ros::Time &time) {
    // nothig to do
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // validate the packet
    if (!packet_.getStart()) {
      ROS_INFO("No packet to be published");
      return;
    }
    if (packet_.getStamp() == last_stamp_) {
      ROS_INFO("Packet has already been published");
      return;
    }

    // publish the packet
    const sensor_msgs::CompressedImagePtr msg(new sensor_msgs::CompressedImage());
    msg->header.stamp = packet_.getStamp();
    msg->format = format_;
    msg->data.assign(packet_.getStartAs< uint8_t >(),
                     packet_.getStartAs< uint8_t >() + packet_.getLength());
    publisher_.publish(msg);
    last_stamp_ = packet_.getStamp();
  }

  virtual void stopping(const ros::Time &time) {
    // nothing to do
  }

private:
  std::string format_;

  usb_cam_hardware_interface::PacketHandle packet_;
  ros::Publisher publisher_;
  ros::Time last_stamp_;
};

} // namespace usb_cam_controllers

#endif