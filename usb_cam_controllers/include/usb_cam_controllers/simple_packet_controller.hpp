#ifndef USB_CAM_CONTROLLERS_SIMPLE_PACKET_CONTROLLER
#define USB_CAM_CONTROLLERS_SIMPLE_PACKET_CONTROLLER

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <usb_cam_hardware_interface/packet_interface.hpp>

namespace usb_cam_controllers {

// a base class for simple packet controllers which
//   - only use the primary interface of packets
//   - expect updated with every new packet
class SimplePacketController
    : public controller_interface::Controller< usb_cam_hardware_interface::PacketInterface > {
public:
  SimplePacketController() {}

  virtual ~SimplePacketController() {}

  virtual bool init(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    // grab the primary interface of packets
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
      ROS_WARN_STREAM(names.size() << " packet handles. camera info synchronized to stamps from "
                                      "the first handle will be published.");
    }
    packet_iface_ = hw->getHandle(names.front());

    last_stamp_ = ros::Time(0);

    // init the child controller
    return initImpl(hw, root_nh, controller_nh);
  }

  virtual void starting(const ros::Time &time) { startingImpl(time); }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // abort if the packet has already been processed
    if (!packet_iface_.getStart()) {
      ROS_DEBUG("No packet. Will skip publishing camera info.");
      return;
    }
    if (packet_iface_.getStamp() == last_stamp_) {
      ROS_DEBUG("Packet is not updated. Will skip publishing camera info.");
      return;
    }

    // process the packet by the child controller
    updateImpl(time, period);

    last_stamp_ = packet_iface_.getStamp();
  }

  virtual void stopping(const ros::Time &time) { stoppingImpl(time); }

protected:
  // init child-controller-specific things. no need to grab the primary packet interface
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) = 0;

  virtual void startingImpl(const ros::Time &time) = 0;

  // update the child controller. called when every new packet comes.
  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) = 0;

  virtual void stoppingImpl(const ros::Time &time) = 0;

protected:
  // the primary packet interface accessible by the child controler
  usb_cam_hardware_interface::PacketHandle packet_iface_;

private:
  ros::Time last_stamp_;
};

} // namespace usb_cam_controllers

#endif