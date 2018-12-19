#ifndef USB_CAM_CONTROLLERS_CAMERA_INFO_CONTROLLER
#define USB_CAM_CONTROLLERS_CAMERA_INFO_CONTROLLER

#include <string>

#include <camera_info_manager/camera_info_manager.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/CameraInfo.h>
#include <usb_cam_controllers/simple_packet_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace usb_cam_controllers {

class CameraInfoController : public SimplePacketController {
public:
  CameraInfoController() {}

  virtual ~CameraInfoController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    frame_id_ = controller_nh.param< std::string >("camera_frame_id", "head_camera");

    publisher_ = controller_nh.advertise< sensor_msgs::CameraInfo >("camera_info", 1);
    info_manager_ = boost::make_shared< camera_info_manager::CameraInfoManager >(
        controller_nh, controller_nh.param< std::string >("camera_name", "head_camera"),
        controller_nh.param< std::string >("camera_info_url", ""));

    return true;
  }

  virtual void startingImpl(const ros::Time &time) {}

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // publish the camera info
    const sensor_msgs::CameraInfoPtr msg(new sensor_msgs::CameraInfo());
    *msg = info_manager_->getCameraInfo();
    msg->header.stamp = packet_iface_.getStamp();
    msg->header.frame_id = frame_id_;
    publisher_.publish(msg);
  }

  virtual void stoppingImpl(const ros::Time &time) {}

private:
  std::string frame_id_;

  ros::Publisher publisher_;
  boost::shared_ptr< camera_info_manager::CameraInfoManager > info_manager_;
};

} // namespace usb_cam_controllers

#endif