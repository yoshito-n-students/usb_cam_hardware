#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include "usb_cam_controllers/camera_info_controller.hpp"
#include "usb_cam_controllers/decoding_controllers.hpp"
#include "usb_cam_controllers/format_controllers.hpp"
#include "usb_cam_controllers/packet_controllers.hpp"

PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::CameraInfoController,
                       controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::CompressedPacketController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::H264Controller, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::MjpegController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::PacketController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::RGB24Controller, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::UYVYController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(usb_cam_controllers::YUYVController, controller_interface::ControllerBase);
