#include <pluginlib/class_list_macros.h>

#include "usb_cam_controllers/camera_info_controller.hpp"
#include "usb_cam_controllers/packet_controller.hpp"

PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::PacketController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::CameraInfoController,
                       controller_interface::ControllerBase);
