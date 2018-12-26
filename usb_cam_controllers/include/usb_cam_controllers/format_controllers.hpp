#ifndef USB_CAM_CONTROLLERS_FORMAT_CONTROLLERS
#define USB_CAM_CONTROLLERS_FORMAT_CONTROLLERS

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>
#include <usb_cam_controllers/simple_packet_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace usb_cam_controllers {

template < cv::ColorConversionCodes ConversionCode, const std::string *DstEncoding >
class FormatController : public SimplePacketController {
public:
  FormatController() {}

  virtual ~FormatController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    width_ = controller_nh.param("image_width", 640);
    height_ = controller_nh.param("image_height", 480);

    // init publisher for decoded images
    publisher_ = image_transport::ImageTransport(controller_nh).advertise("image", 1);

    return true;
  }

  virtual void startingImpl(const ros::Time &time) {
    // nothing to do
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // allocate output message
    cv_bridge::CvImage out;
    out.header.stamp = packet_iface_.getStamp();
    out.encoding = *DstEncoding;

    // convert pixel formats
    try {
      cv::cvtColor(cv::Mat(height_, width_, CV_8UC(packet_iface_.getLength() / (height_ * width_)),
                           const_cast< uint8_t * >(packet_iface_.getStartAs< uint8_t >())),
                   out.image, ConversionCode);
    } catch (const cv::Exception &ex) {
      ROS_ERROR_STREAM(ex.what());
      return;
    }

    publisher_.publish(out.toImageMsg());
  }

  virtual void stoppingImpl(const ros::Time &time) {
    // nothing to do
  }

private:
  int height_, width_;

  image_transport::Publisher publisher_;
};

typedef FormatController< cv::COLOR_RGB2BGR, &sensor_msgs::image_encodings::BGR8 >
    RGB24Controller;
typedef FormatController< cv::COLOR_YUV2BGR_UYVY, &sensor_msgs::image_encodings::BGR8 >
    UYVYController;
typedef FormatController< cv::COLOR_YUV2BGR_YUYV, &sensor_msgs::image_encodings::BGR8 >
    YUYVController;

} // namespace usb_cam_controllers

#endif