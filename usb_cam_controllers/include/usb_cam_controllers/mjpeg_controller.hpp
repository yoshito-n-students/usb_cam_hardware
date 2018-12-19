#ifndef USB_CAM_CONTROLLERS_MJPEG_CONTROLLER
#define USB_CAM_CONTROLLERS_MJPEG_CONTROLLER

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <usb_cam_controllers/simple_packet_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

namespace usb_cam_controllers {

class MjpegController : public SimplePacketController {
public:
  MjpegController() {}

  virtual ~MjpegController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    encoding_ = controller_nh.param< std::string >("encoding", "bgr8");

    publisher_ = image_transport::ImageTransport(controller_nh).advertise("image", 1);

    return true;
  }

  virtual void startingImpl(const ros::Time &time) {
    // nothig to do
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // decode & publish the frame
    cv_bridge::CvImage image;
    image.header.stamp = packet_iface_.getStamp();
    image.encoding = encoding_;
    image.image = cv::imdecode(
        cv::_InputArray(packet_iface_.getStartAs< uint8_t >(), packet_iface_.getLength()),
        -1 /* decode the data as is */);
    publisher_.publish(image.toImageMsg());
  }

  virtual void stoppingImpl(const ros::Time &time) {
    // nothing to do
  }

private:
  std::string encoding_;

  image_transport::Publisher publisher_;
};

} // namespace usb_cam_controllers

#endif