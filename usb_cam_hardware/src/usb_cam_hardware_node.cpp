#include <controller_manager/controller_manager.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <ros/time.h>
#include <usb_cam_hardware/usb_cam_hardware.hpp>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "usb_cam_hardware");
  ros::NodeHandle nh, pnh("~");

  usb_cam_hardware::USBCamHardware hardware;

  controller_manager::ControllerManager controllers(&hardware, nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate control_rate(nh.param("fps", 30.));
  ros::Time last(ros::Time::now());
  while (ros::ok()) {
    const ros::Time now(ros::Time::now());
    const ros::Duration period(now - last);
    hardware.read(now, period);
    controllers.update(now, period);
    hardware.write(now, period);
    last = now;
    control_rate.sleep();
  }

  return 0;
}