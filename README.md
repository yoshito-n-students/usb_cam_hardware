# usb_cam_hardware
USB camera driver based on ros_control

[Slides from ROS Japan UG #28](https://drive.google.com/file/d/15ueFzd4BfJbSNH38vfEatP0qbntPlJCy/view) (in Japanese)

## Nodelet: usb_cam_hardware/USBCamHardware
An USB camera driver which is almost compatible to [usb_cam](http://wiki.ros.org/usb_cam) but based on [ros_control](http://wiki.ros.org/ros_control). Major enhancements over usb_cam are;
* impremented as a nodelet
* supports h264 pixel format (1/10 network usage compared with mjpeg)
* publishes raw packets from the camera (1/10 cpu usage and less latency compared with decoding mjpeg packets in the node)
* accepts 3rd party controllers

### Published topics
depends on [controllers](#Controllers)

### Parameters
**~video_device** (string, default: "/dev/video0")

**~image_width** (int, default: 640)

**~image_height** (int, default: 480)

**~pixel_format** (string, default: "mjpeg")
* Possible values are mjpeg, h264, rgb24, yuyv, uyvy

**~framerate** (int, default: 30)

## Controllers
* All topics and parameters are defined in each controller's namespace
* See example launch files to find how to use controllers

### CameraInfoController
publishes camera information syncronized to packets from the camera

#### Published topics
**camera_info** (sensor_msgs/CameraInfo)

#### Parameters
**camera_frame_id** (string, default: "head_camera")

**camera_info_url** (string, default: "")

**camera_name** (string, default: "head_camera")

### PacketController
publishes raw packets from the camera as images

#### Published topics
**image** (sensor_msgs/Image)

#### Parameters
**image_width** (int, default: 640)

**image_height** (int, default: 480)

**encoding** (string, default: "bgr8")
* value of Image::encoding

**skip** (int, default: 0)
* number of packets skipped after publishment
* useful to throttle network usage

### CompressedPacketController
publishes raw packets from the camera as compressed images

#### Published topics
**packet** (sensor_msgs/CompressedImage)

#### Parameters
**format** (string, default: "jpeg")
* value of CompressedImage::format

**skip** (int, default: 0)
* number of packets skipped after publishment
* useful to throttle network usage

### MjpegController
decodes mjpeg-compressed packets and publishes decoded images

#### Published topics
**image** (sensor_msgs/Image)

### H264Controller
decodes h264-compressed packets and publishes decoded images

#### Published topics
**image** (sensor_msgs/Image)

### RGB24Controller
converts rgb-formatted packets to bgr images and publishes them

#### Published topics
**image** (sensor_msgs/Image)

#### Parameters
**image_width** (int, default: 640)

**image_height** (int, default: 480)

### UYVYController
converts uyvy-formatted packets to bgr images and publishes them

#### Published topics
**image** (sensor_msgs/Image)

#### Parameters
**image_width** (int, default: 640)

**image_height** (int, default: 480)

### YUYVController 
converts yuyv-formatted packets to bgr images and publishes them

#### Published topics
**image** (sensor_msgs/Image)

#### Parameters
**image_width** (int, default: 640)

**image_height** (int, default: 480)

## Related ROS packages
[codec_image_transport](https://github.com/yoshito-n-students/codec_image_transport)
* for subscriber-side decoding of h264 packets published by the CompressedPacketController