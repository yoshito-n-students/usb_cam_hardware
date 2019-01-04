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

  // init the camera and get the framerate which the camera is operated at
  usb_cam_hardware::USBCamHardware hardware;
  const ros::Duration time_per_frame(hardware.init(pnh));
  if (time_per_frame <= ros::Duration(0.)) {
    ROS_ERROR("Cannot initialize the usb cam hardware");
    return 1;
  }

  // bind controller manager and the camera
  controller_manager::ControllerManager controllers(&hardware, nh);
  ros::AsyncSpinner spinner(1); // to exec callbacks of controller management services
  spinner.start();

  // run control loops
  ros::Time last(ros::Time::now());
  ros::Rate rate(time_per_frame);
  while (ros::ok()) {
    const ros::Time now(ros::Time::now());
    const ros::Duration period(now - last);
    hardware.read(now, period);
    controllers.update(now, period);
    hardware.write(now, period);
    last = now;
    rate.sleep();
  }

  return 0;
}