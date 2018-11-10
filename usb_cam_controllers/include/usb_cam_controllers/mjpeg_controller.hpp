#ifndef USB_CAM_CONTROLLERS_MJPEG_CONTROLLER
#define USB_CAM_CONTROLLERS_MJPEG_CONTROLLER

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace usb_cam_controllers {

class MjpegController
    : public controller_interface::Controller< usb_cam_hardware_interface::PacketInterface > {
public:
  MjpegController() {}

  virtual bool init(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    encoding_ = controller_nh.param< std::string >("encoding", "bgr8");

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
    publisher_ = image_transport::ImageTransport(controller_nh).advertise("image", 1);
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
    cv_bridge::CvImage image;
    image.header.stamp = packet_.getStamp();
    image.encoding = encoding_;
    image.image = cv::imdecode(cv::_InputArray(packet_.getStart(), packet_.getLength()),
                               -1 /* decode the data as is */);
    publisher_.publish(image.toImageMsg());
    last_stamp_ = packet_.getStamp();
  }

  virtual void stopping(const ros::Time &time) {
    // nothing to do
  }

private:
  std::string encoding_;

  usb_cam_hardware_interface::PacketHandle packet_;
  image_transport::Publisher publisher_;
  ros::Time last_stamp_;
};

} // namespace usb_cam_controllers

#endif