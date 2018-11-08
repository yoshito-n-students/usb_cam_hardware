#ifndef USB_CAM_HARDWARE_USB_CAM_HARDWARE
#define USB_CAM_HARDWARE_USB_CAM_HARDWARE

#include <cstring> // for std::memset()
#include <string>
#include <vector>

#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <usb_cam_hardware_interface/packet_interface.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <linux/videodev2.h>

namespace usb_cam_hardware {

class USBCamHardware : public hardware_interface::RobotHW {
public:
  USBCamHardware() : fd_(-1) {}
  virtual ~USBCamHardware() {
    // TODO: stop streaming and close device
  }

  bool init(ros::NodeHandle param_nh) {
    // register the packet buffer to the interface so that controllers can see the packet
    {
      packet_.start = NULL;
      packet_.length = 0;
      packet_.buffer_index = -1;
      packet_interface_.registerHandle(
          usb_cam_hardware_interface::PacketHandle("packet", &packet_.start, &packet_.length));
      registerInterface(&packet_interface_);
    }

    // open the device
    {
      const std::string video_device(param_nh.param< std::string >("video_device", "/dev/video0"));
      fd_ = open(video_device.c_str(), O_RDWR | O_NONBLOCK);
      if (fd_ < 0) {
        ROS_ERROR_STREAM("Cannot open \"" << video_device << "\"");
        return false;
      }
    }

    // TODO: check device capability

    // cancel device-side cropping and scaling
    {
      v4l2_cropcap cropcap;
      std::memset(&cropcap, 0, sizeof(cropcap));
      // get the cropping capability
      if (ioctl(fd_, VIDIOC_CROPCAP, &cropcap) == 0) {
        // set the default (no) cropping
        v4l2_crop crop;
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect;
        ioctl(fd_, VIDIOC_S_CROP, &crop);
      }
    }

    // set image format
    {
      v4l2_format format;
      std::memset(&format, 0, sizeof(format));
      format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      format.fmt.pix.width = param_nh.param("image_width", 640);
      format.fmt.pix.height = param_nh.param("image_height", 480);
      const std::string pixel_format(param_nh.param< std::string >("pixel_format", "mpjeg"));
      if (pixel_format == "grey") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
      } else if (pixel_format == "h264") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
      } else if (pixel_format == "mjpeg") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
      } else if (pixel_format == "rgb24") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      } else if (pixel_format == "uyvy") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      } else if (pixel_format == "yuyv") {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      } else {
        ROS_ERROR_STREAM("Pixel format \"" << pixel_format << "\" is not supported");
        return false;
      }
      format.fmt.pix.field = V4L2_FIELD_INTERLACED;
      if (ioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
        ROS_ERROR("Cannot set format");
        return false;
      }
    }

    // set framerate
    {
      v4l2_streamparm streamparm;
      std::memset(&streamparm, 0, sizeof(streamparm));
      streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (ioctl(fd_, VIDIOC_G_PARM, &streamparm) < 0) {
        ROS_ERROR("Cannot get streaming parameters");
        return false;
      }
      streamparm.parm.capture.timeperframe.numerator = 1;
      streamparm.parm.capture.timeperframe.denominator = param_nh.param("framerate", 30);
      if (ioctl(fd_, VIDIOC_S_PARM, &streamparm) < 0) {
        ROS_ERROR("Cannot set framerate");
        return false;
      }
    }

    // allocate buffers
    // TODO: support not only mmap, but also userp and read
    {
      v4l2_requestbuffers reqbufs;
      std::memset(&reqbufs, 0, sizeof(reqbufs));
      reqbufs.count = 4;
      reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      reqbufs.memory = V4L2_MEMORY_MMAP;
      if (ioctl(fd_, VIDIOC_REQBUFS, &reqbufs) < 0) {
        ROS_ERROR("Cannot request buffers");
        return false;
      }
      if (reqbufs.count < 2) {
        ROS_ERROR("Insufficient buffer memory on the device");
        return false;
      }

      for (std::size_t i = 0; i < reqbufs.count; ++i) {
        v4l2_buffer buffer;
        std::memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (ioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
          ROS_ERROR("Cannot query buffer");
          return false;
        }

        Buffer mapped_buffer;
        mapped_buffer.start = mmap(NULL /* start anywhere */, buffer.length, PROT_READ | PROT_WRITE,
                                   MAP_SHARED, fd_, buffer.m.offset);
        mapped_buffer.length = buffer.length;
        if (mapped_buffer.start == MAP_FAILED) {
          ROS_ERROR("Cannot map memory");
          return false;
        }

        buffers_.push_back(mapped_buffer);
      }
    }

    // start streaming
    {
      for (std::size_t i = 0; i < buffers_.size(); ++i) {
        v4l2_buffer buffer;
        std::memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;
        if (ioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
          ROS_ERROR("Cannot enqueue buffer");
          return false;
        }
      }

      v4l2_buf_type buf_type;
      buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (ioctl(fd_, VIDIOC_STREAMON, &buf_type) < 0) {
        ROS_ERROR("Cannot start streaming");
        return false;
      }
    }

    return true;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    //
    if (packet_.buffer_index >= 0) {
      ROS_ERROR("last packet is not cleared. call write() first.");
      return;
    }

    // pop buffer
    v4l2_buffer buffer;
    std::memset(&buffer, 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
      switch (errno) {
      case EAGAIN:
        /* */
        return;
      case EIO:
        // TODO: code here is just copied from the original usb_cam.
        //       understand why falling through is ok.
        break;
      default:
        ROS_ERROR("Cannot dequeue buffer");
        return;
      }
    }

    // fill packet info with the poped buffer
    packet_.start = buffers_[buffer.index].start;
    packet_.length = buffer.bytesused;
    packet_.buffer_index = buffer.index;
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // push buffer
    if (packet_.buffer_index >= 0) {
      v4l2_buffer buffer;
      std::memset(&buffer, 0, sizeof(buffer));
      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;
      buffer.index = packet_.buffer_index;
      if (ioctl(fd_, VIDIOC_QBUF, &buffer) == 0) {
        packet_.start = NULL;
        packet_.length = 0;
        packet_.buffer_index = -1;
      } else {
        ROS_ERROR("Cannot enqueue buffer");
      }
    }
  }

private:
  struct Packet {
    const void *start;
    std::size_t length;
    int buffer_index;
  };
  struct Buffer {
    const void *start;
    std::size_t length;
  };

  int fd_;

  usb_cam_hardware_interface::PacketInterface packet_interface_;
  Packet packet_;

  std::vector< Buffer > buffers_;
};

} // namespace usb_cam_hardware

#endif