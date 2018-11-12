#ifndef USB_CAM_HARDWARE_INTERFACE_PACKET_INTERFACE
#define USB_CAM_HARDWARE_INTERFACE_PACKET_INTERFACE

#include <string>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <ros/time.h>

namespace usb_cam_hardware_interface {

// requirement for a hardware handle
//  - copyable to be mapped in HardwareResourceManager
//  - has getName() to be used in HardwareResouceManager
//  - has reference to data
class PacketHandle {
public:
  PacketHandle() : name_(), stamp_(NULL), start_(NULL), length_(NULL) {}
  PacketHandle(const std::string &name, const ros::Time *const stamp, const void **const start,
               const std::size_t *const length)
      : name_(name), stamp_(stamp), start_(start), length_(length) {}
  virtual ~PacketHandle() {}

  std::string getName() const { return name_; }
  ros::Time getStamp() const { return *stamp_; }
  const void *getStart() const { return *start_; }
  template < typename Byte > const Byte *getStartAs() const {
    return reinterpret_cast< const Byte * >(*start_);
  };
  std::size_t getLength() const { return *length_; }

private:
  std::string name_;
  const ros::Time *stamp_;
  const void **start_;
  const std::size_t *length_;
};

class PacketInterface : public hardware_interface::HardwareResourceManager< PacketHandle > {};

} // namespace usb_cam_hardware_interface

#endif