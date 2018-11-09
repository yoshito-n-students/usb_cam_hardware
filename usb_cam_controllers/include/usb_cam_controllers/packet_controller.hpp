#ifndef USB_CAM_CONTROLLERS_PACKET_CONTROLLER
#define USB_CAM_CONTROLLERS_PACKET_CONTROLLER

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <usb_cam_hardware_interface/packet_interface.hpp>

namespace usb_cam_controllers {

class PacketController
    : public controller_interface::Controller< usb_cam_hardware_interface::PacketInterface > {
public:
  PacketController() {}

  virtual bool init(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    return true;
  }

  virtual void starting(const ros::Time &time) {}

  virtual void update(const ros::Time &time, const ros::Duration &period) {}

  virtual void stopping(const ros::Time &time) {}

private:
};

} // namespace usb_cam_controllers

#endif