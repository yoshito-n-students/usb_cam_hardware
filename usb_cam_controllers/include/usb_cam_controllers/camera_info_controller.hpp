#ifndef USB_CAM_CONTROLLERS_CAMERA_INFO_CONTROLLER
#define USB_CAM_CONTROLLERS_CAMERA_INFO_CONTROLLER

#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.h>
#include <controller_interface/controller.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace usb_cam_controllers {

class CameraInfoController
    : public controller_interface::Controller< usb_cam_hardware_interface::PacketInterface > {
public:
  CameraInfoController() {}

  virtual bool init(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    frame_id_ = controller_nh.param< std::string >("camera_frame_id", "head_camera");

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

    packet_ = hw->getHandle(names.front());
    publisher_ = root_nh.advertise< sensor_msgs::CameraInfo >("camera_info", 1);
    info_manager_ = boost::make_shared< camera_info_manager::CameraInfoManager >(
        root_nh, controller_nh.param< std::string >("camera_name", "head_camera"),
        controller_nh.param< std::string >("camera_info_url", ""));
    last_stamp_ = ros::Time(0);

    return true;
  }

  virtual void starting(const ros::Time &time) {
    // nothing to do
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // validate the packet
    if (!packet_.getStart()) {
      ROS_INFO("No packet. Will skip publishing camera info.");
      return;
    }
    if (packet_.getStamp() == last_stamp_) {
      ROS_INFO("Packet is not updated. Will skip publishing camera info.");
      return;
    }

    // publish the camera info
    const sensor_msgs::CameraInfoPtr msg(new sensor_msgs::CameraInfo());
    *msg = info_manager_->getCameraInfo();
    msg->header.stamp = packet_.getStamp();
    msg->header.frame_id = frame_id_;
    publisher_.publish(msg);
    last_stamp_ = packet_.getStamp();
  }

  virtual void stopping(const ros::Time &time) {
    // nothing to do
  }

private:
  std::string frame_id_;

  usb_cam_hardware_interface::PacketHandle packet_;
  ros::Publisher publisher_;
  boost::shared_ptr< camera_info_manager::CameraInfoManager > info_manager_;
  ros::Time last_stamp_;
};

} // namespace usb_cam_controllers

#endif