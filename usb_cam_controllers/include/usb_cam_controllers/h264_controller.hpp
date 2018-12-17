#ifndef USB_CAM_CONTROLLERS_H264_CONTROLLER
#define USB_CAM_CONTROLLERS_H264_CONTROLLER

#include <controller_interface/controller.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <usb_cam_hardware_interface/packet_interface.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace usb_cam_controllers {

class H264Controller
    : public controller_interface::Controller< usb_cam_hardware_interface::PacketInterface > {
public:
  H264Controller() {}

  virtual ~H264Controller() {}

  virtual bool init(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    //
    // grab a interface for h264-compressed packet between hardware
    //

    {
      if (!hw) {
        ROS_ERROR("Null packet interface");
        return false;
      }

      const std::vector< std::string > names(hw->getNames());
      if (names.empty()) {
        ROS_ERROR("No packet handle");
        return false;
      }
      if (names.size() > 1) {
        ROS_WARN_STREAM(names.size() << " packet handles. camera info synchronized to stamps from "
                                        "the first handle will be published.");
      }

      packet_iface_ = hw->getHandle(names.front());
    }

    //
    // init a h264 decoder
    //

    {
      // init libavformat
      avcodec_register_all();
      av_log_set_level(AV_LOG_FATAL);

      // find h264 decoder
      AVCodec *const decoder(avcodec_find_decoder(AV_CODEC_ID_H264));
      if (!decoder) {
        ROS_ERROR("Cannot find h264 decoder");
        return false;
      }

      // allocate h264 decoder context
      decoder_ctx_.reset(avcodec_alloc_context3(decoder), AVDeleter());
      if (!decoder_ctx_) {
        ROS_ERROR("Cannot allocate h264 decoder context");
        return false;
      }

      // open decoder
      if (avcodec_open2(decoder_ctx_.get(), decoder, NULL) < 0) {
        ROS_ERROR("Failed to open h264 codec");
        return false;
      }
    }

    //
    // init publisher for decoded images
    //

    publisher_ = image_transport::ImageTransport(controller_nh).advertise("image", 1);
    last_stamp_ = ros::Time(0);

    return true;
  }

  virtual void starting(const ros::Time &time) {
    // nothing to do
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    // validate the packet
    if (!packet_iface_.getStart()) {
      ROS_INFO("No packet. Will skip publishing camera info.");
      return;
    }
    if (packet_iface_.getStamp() == last_stamp_) {
      ROS_INFO("Packet is not updated. Will skip publishing camera info.");
      return;
    }

    // set packet data from the input message
    AVPacket packet;
    av_init_packet(&packet);
    packet.size = packet_iface_.getLength();
    packet.data = const_cast< uint8_t * >(packet_iface_.getStartAs< uint8_t >());

    // send the packet to the decoder
    if (avcodec_send_packet(decoder_ctx_.get(), &packet) < 0) {
      ROS_ERROR("Cannot send h264 packet to decoder");
      return;
    }

    while (true) {
      // allocate a frame for decoded data
      boost::shared_ptr< AVFrame > frame(av_frame_alloc(), AVDeleter());
      if (!frame) {
        ROS_ERROR("Cannot allocate frame");
        return;
      }

      // receive the decoded data from the decoder
      const int res(avcodec_receive_frame(decoder_ctx_.get(), frame.get()));
      if (res == AVERROR(EAGAIN) || res == AVERROR_EOF) {
        // no more frames in the packet
        return;
      } else if (res < 0) {
        ROS_ERROR("Cannot receive h264 frame");
        return;
      }

      // allocate output message
      const sensor_msgs::ImagePtr out(new sensor_msgs::Image());
      out->header.stamp = packet_iface_.getStamp();
      out->height = frame->height;
      out->width = frame->width;
      out->encoding = sensor_msgs::image_encodings::BGR8;
      out->step = 3 * frame->width;
      out->data.resize(3 * frame->width * frame->height);

      // layout data by converting color spaces (YUV -> RGB)
      boost::shared_ptr< SwsContext > convert_ctx(sws_getContext(
                                                      // src formats
                                                      frame->width, frame->height,
                                                      AV_PIX_FMT_YUV420P,
                                                      // dst formats
                                                      frame->width, frame->height, AV_PIX_FMT_BGR24,
                                                      // flags & filters
                                                      SWS_FAST_BILINEAR, NULL, NULL, NULL),
                                                  AVDeleter());
      int stride = 3 * frame->width;
      uint8_t *dst = &out->data[0];
      sws_scale(convert_ctx.get(),
                // src data
                frame->data, frame->linesize, 0, frame->height,
                // dst data
                &dst, &stride);

      publisher_.publish(out);
      last_stamp_ = packet_iface_.getStamp();
    }
  }

  virtual void stopping(const ros::Time &time) {
    // nothing to do
  }

private:
  // Deleter for auto free/close of libav objects
  struct AVDeleter {
    void operator()(AVFrame *frame) {
      if (frame) {
        av_frame_free(&frame);
      }
    }

    void operator()(AVCodecContext *ctx) {
      if (ctx) {
        avcodec_free_context(&ctx);
      }
    }

    void operator()(SwsContext *ctx) {
      if (ctx) {
        sws_freeContext(ctx);
      }
    }
  };

private:
  usb_cam_hardware_interface::PacketHandle packet_iface_;
  boost::shared_ptr< AVCodecContext > decoder_ctx_;
  image_transport::Publisher publisher_;
  ros::Time last_stamp_;
};

} // namespace usb_cam_controllers

#endif