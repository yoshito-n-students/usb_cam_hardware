#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include "usb_cam_hardware/usb_cam_hardware_nodelet.hpp"

PLUGINLIB_EXPORT_CLASS(usb_cam_hardware::USBCamHardwareNodelet, nodelet::Nodelet);
