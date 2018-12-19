#ifndef USB_CAM_CONTROLLERS_PACKET_CONTROLLER
#define USB_CAM_CONTROLLERS_PACKET_CONTROLLER

#include <string>

#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <sensor_msgs/CompressedImage.h>
#include <usb_cam_controllers/simple_packet_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

namespace usb_cam_controllers {

class PacketController : public SimplePacketController {
public:
  PacketController() {}

  virtual ~PacketController() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    format_ = controller_nh.param< std::string >("format", "");

    publisher_ = controller_nh.advertise< sensor_msgs::CompressedImage >("packet", 1);

    return true;
  }

  virtual void startingImpl(const ros::Time &time) {
    // nothig to do
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // publish the packet
    const sensor_msgs::CompressedImagePtr msg(new sensor_msgs::CompressedImage());
    msg->header.stamp = packet_iface_.getStamp();
    msg->format = format_;
    msg->data.assign(packet_iface_.getStartAs< uint8_t >(),
                     packet_iface_.getStartAs< uint8_t >() + packet_iface_.getLength());
    publisher_.publish(msg);
  }

  virtual void stoppingImpl(const ros::Time &time) {
    // nothing to do
  }

private:
  std::string format_;

  ros::Publisher publisher_;
};

} // namespace usb_cam_controllers

#endif