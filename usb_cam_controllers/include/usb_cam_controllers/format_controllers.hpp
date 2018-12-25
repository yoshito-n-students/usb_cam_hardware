#ifndef USB_CAM_CONTROLLERS_FORMAT_CONTROLLERS
#define USB_CAM_CONTROLLERS_FORMAT_CONTROLLERS

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <usb_cam_controllers/simple_packet_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <opencv2/imgproc/imgproc.hpp>

namespace usb_cam_controllers {

template < cv::ColorConversionCodes ConversionCode >
class FormatController : public SimplePacketController {
public:
  FormatController() {}

  virtual ~FormatController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    width_ = controller_nh.param("image_width", 640);
    height_ = controller_nh.param("image_height", 480);
    encoding_ = controller_nh.param< std::string >("encoding", sensor_msgs::image_encodings::BGR8);

    return true;
  }

  virtual void startingImpl(const ros::Time &time) {
    // nothing to do
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // allocate output message
    const sensor_msgs::ImagePtr out(new sensor_msgs::Image());
    out->header.stamp = packet_iface_.getStamp();
    out->height = height_;
    out->width = width_;
    out->encoding = encoding_;
    out->step = 3 * width_;

    // convert pixel formats. dst format must be one of 24-bit RGB (BGR, RGB, etc)
    cv::cvtColor(cv::_InputArray(packet_iface_.getStartAs< uint8_t >(), packet_iface_.getLength()),
                 out->data, ConversionCode);

    publisher_.publish(out);
  }

  virtual void stoppingImpl(const ros::Time &time) {
    // nothing to do
  }

private:
  std::string encoding_;
  int height_, width_;

  image_transport::Publisher publisher_;
}; // namespace usb_cam_controllers

typedef FormatController< cv::COLOR_YUV2BGR_UYVY > UYVYController;
typedef FormatController< cv::COLOR_YUV2BGR_YUYV > YUYVController;

} // namespace usb_cam_controllers

#endif