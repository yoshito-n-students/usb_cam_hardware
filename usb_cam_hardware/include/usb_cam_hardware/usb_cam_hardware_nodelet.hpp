#ifndef USB_CAM_HARDWARE_USB_CAM_HARDWARE_NODELET
#define USB_CAM_HARDWARE_USB_CAM_HARDWARE_NODELET

#include <controller_manager/controller_manager.h>
#include <nodelet/nodelet.h>
#include <ros/duration.h>
#include <ros/exception.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <usb_cam_hardware/usb_cam_hardware.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace usb_cam_hardware {

class USBCamHardwareNodelet : public nodelet::Nodelet {
public:
  USBCamHardwareNodelet() {}

  virtual ~USBCamHardwareNodelet() {
    update_timer_.stop(); // stop updating the controllers and hardware first
    controllers_.reset(); // unload the controllers using the hardware
    hardware_.reset();    // finally close the hardware
  }

private:
  virtual void onInit() {
    ros::NodeHandle nh(getMTNodeHandle()), pnh(getMTPrivateNodeHandle());

    hardware_ = boost::make_shared< USBCamHardware >();
    const ros::Duration time_per_frame(hardware_->init(pnh));
    if (time_per_frame <= ros::Duration(0.)) {
      ROS_ERROR("Cannot init usb cam hardware");
      throw ros::Exception("Cannot init usb cam hardware");
    }

    controllers_ = boost::make_shared< controller_manager::ControllerManager >(hardware_.get(), nh);

    update_timer_ = nh.createTimer(time_per_frame, &USBCamHardwareNodelet::update, this);

    last_ = ros::Time::now();
  }

  void update(const ros::TimerEvent &event) {
    const ros::Time now(ros::Time::now());
    const ros::Duration period(now - last_);
    hardware_->read(now, period);
    controllers_->update(now, period);
    hardware_->write(now, period);
    last_ = now;
  }

private:
  boost::shared_ptr< USBCamHardware > hardware_;
  boost::shared_ptr< controller_manager::ControllerManager > controllers_;
  ros::Timer update_timer_;

  ros::Time last_;
};

} // namespace usb_cam_hardware

#endif