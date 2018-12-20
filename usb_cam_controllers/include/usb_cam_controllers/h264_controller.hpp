#ifndef USB_CAM_CONTROLLERS_H264_CONTROLLER
#define USB_CAM_CONTROLLERS_H264_CONTROLLER

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <usb_cam_controllers/simple_packet_controller.hpp>
#include <usb_cam_hardware_interface/packet_interface.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace usb_cam_controllers {

class H264Controller : public SimplePacketController {
public:
  H264Controller() {}

  virtual ~H264Controller() {}

protected:
  virtual bool initImpl(usb_cam_hardware_interface::PacketInterface *hw, ros::NodeHandle &root_nh,
                        ros::NodeHandle &controller_nh) {
    // init libavcodec
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

    encoding_ = controller_nh.param< std::string >("encoding", sensor_msgs::image_encodings::BGR8);

    // init publisher for decoded images
    publisher_ = image_transport::ImageTransport(controller_nh).advertise("image", 1);

    return true;
  }

  virtual void startingImpl(const ros::Time &time) {
    // nothing to do
  }

  virtual void updateImpl(const ros::Time &time, const ros::Duration &period) {
    // set packet data from the input message
    AVPacket packet;
    av_init_packet(&packet);
    packet.size = packet_iface_.getLength();
    packet.data = const_cast< uint8_t * >(packet_iface_.getStartAs< uint8_t >());

    // repeat decoding until all data in the packet are consumed
    while (packet.size > 0) {
      // decode one frame
      boost::shared_ptr< AVFrame > frame(av_frame_alloc(), AVDeleter());
      int got_frame;
      const int len(avcodec_decode_video2(decoder_ctx_.get(), frame.get(), &got_frame, &packet));
      if (len < 0) {
        ROS_ERROR("Cannot decode a frame");
        return;
      }

      // publish the decoded frame
      if (got_frame > 0) {
        // allocate output message
        const sensor_msgs::ImagePtr out(new sensor_msgs::Image());
        out->header.stamp = packet_iface_.getStamp();
        out->height = frame->height;
        out->width = frame->width;
        out->encoding = sensor_msgs::image_encodings::BGR8;
        out->step = 3 * frame->width;
        out->data.resize(3 * frame->width * frame->height);

        // layout data by converting color spaces (YUV -> RGB)
        boost::shared_ptr< SwsContext > convert_ctx(
            sws_getContext(
                // src formats
                frame->width, frame->height, AV_PIX_FMT_YUV420P,
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
      }

      // consume data in the packet
      packet.size -= len;
      packet.data += len;
    }
  }

  virtual void stoppingImpl(const ros::Time &time) {
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
  std::string encoding_;

  boost::shared_ptr< AVCodecContext > decoder_ctx_;
  image_transport::Publisher publisher_;
};

} // namespace usb_cam_controllers

#endif