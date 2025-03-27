#include <array>
#include <cstring> // for std::memset()
#include <iostream>
#include <numeric> // for std::partial_sum()

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <libav_cpp/libav_cpp.hpp>

namespace libav_cpp {

// =====
// Error
// =====

std::string Error::err2str(const int errnum) {
  char buf[AV_ERROR_MAX_STRING_SIZE];
  av_strerror(errnum, buf, sizeof(buf));
  return buf;
}

// ========================================
// BufferRef - RAII wrapper for AVBufferRef
// ========================================

BufferRef::BufferRef(const std::size_t unpadded_size)
    : buf_(av_buffer_alloc(unpadded_size + AV_INPUT_BUFFER_PADDING_SIZE), &unref_buffer) {
  if (!buf_) {
    throw Error("BufferRef::BufferRef(): Failed to allocate AVBufferRef");
  }
  // Zero the padding area
  std::memset(buf_->data + unpadded_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
}

BufferRef::BufferRef(const BufferRef &buf) : buf_(av_buffer_ref(buf.get()), &unref_buffer) {
  if (!buf_) {
    throw Error("BufferRef::BufferRef(): Failed to create a reference to buffer");
  }
}

void BufferRef::unref_buffer(AVBufferRef *buf) { av_buffer_unref(&buf); }

// ==================================
// Packet - RAII wrapper for AVPacket
// ==================================

Packet::Packet() : packet_(av_packet_alloc(), free_packet) {
  if (!packet_) {
    throw Error("Packet::Packet(): Failed to allocate AVPacket");
  }
}

void Packet::ref(const BufferRef &buf) {
  unref();
  packet_->buf = av_buffer_ref(buf.get());
  if (!packet_->buf) {
    throw Error("Packet::ref(): Failed to create a reference to buffer");
  }
  packet_->data = buf->data;
  packet_->size = buf.unpadded_size();
}

void Packet::unref() { av_packet_unref(packet_.get()); }

void Packet::free_packet(AVPacket *packet) { av_packet_free(&packet); }

// ================================
// Frame - RAII wrapper for AVFrame
// ================================

Frame::Frame() : frame_(av_frame_alloc(), free_frame) {
  if (!frame_) {
    throw Error("Frame::Frame(): Failed to allocate AVFrame");
  }
}

void Frame::unref() { av_frame_unref(frame_.get()); }

void Frame::transfer_data(Frame *const dst) const {
  dst->unref();
  if (const int ret = av_hwframe_transfer_data(dst->get(), frame_.get(), 0); ret < 0) {
    throw Error("Frame::transfer_data(): Error transferring data", ret);
  }
}

void Frame::free_frame(AVFrame *frame) { av_frame_free(&frame); }

// ========================================
// Input - RAII wrapper for AVFormatContext
// ========================================

Input::Input(const std::string &url, const std::string &format_name,
             const std::map<std::string, std::string> &option_map)
    : format_ctx_(nullptr, &close_input) {
  // Register all the input format types
  avdevice_register_all();

  // Find the input format by name
  const AVInputFormat *format =
      (format_name.empty() ? nullptr : av_find_input_format(format_name.c_str()));
  if (!format_name.empty() && format == nullptr) {
    throw Error("Input::Input(): " + format_name + " was not recognized as an input format");
  }

  // Build the option dictionary from the map
  AVDictionary *option_dict = nullptr;
  for (const auto &[key, value] : option_map) {
    if (const int ret = av_dict_set(&option_dict, key.c_str(), value.c_str(), 0); ret < 0) {
      av_dict_free(&option_dict);
      throw Error("Input::Input(): Failed to pack option [" + key + ", " + value + "]", ret);
    }
  }

  // Open the input with the URL, format and options
  AVFormatContext *format_ctx = nullptr;
  if (const int ret = avformat_open_input(&format_ctx, url.c_str(), format, &option_dict);
      ret < 0) {
    throw Error("Input::Input(): Failed to open input " + url, ret);
  }
  format_ctx_.reset(format_ctx);

  // Check if the input accepts all the options
  if (option_dict) {
    AVDictionaryEntry *remaining_option = nullptr;
    av_dict_get(option_dict, "", remaining_option, AV_DICT_IGNORE_SUFFIX);
    const std::string msg = "Input::Input(): Input " + url + " does not accept option [" +
                            remaining_option->key + ", " + remaining_option->value + "]";
    av_dict_free(&option_dict);
    throw Error(msg);
  }

  // Retrieve stream information on the input
  // (TODO: check if this is necessary to find the video stream)
  if (const int ret = avformat_find_stream_info(format_ctx, nullptr); ret < 0) {
    throw Error("Input::Input(): Failed to find stream information", ret);
  }

  // Find the best video stream
  stream_id_ = av_find_best_stream(format_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if (stream_id_ < 0) {
    throw Error("Input::Input(): Failed to find video stream", stream_id_);
  }
}

void Input::read_frame(Packet *const packet) {
  packet->unref();
  do {
    if (const int ret = av_read_frame(format_ctx_.get(), packet->get()); ret < 0) {
      throw Error("Input::read_frame(): Failed to read frame", ret);
    }
  } while ((*packet)->stream_index != stream_id_);
}

void Input::close_input(AVFormatContext *format_ctx) { avformat_close_input(&format_ctx); }

// =========================================
// Decoder - RAII wrapper for AVCodecContext
// =========================================

Decoder::Decoder(const std::string &codec_name) : codec_ctx_(nullptr, &free_context) {
  // Find the decoder by name
  const AVCodec *const codec = avcodec_find_decoder_by_name(codec_name.c_str());
  if (!codec) {
    throw Error("Decoder::Decoder(): " + codec_name + " was not recognized as a decoder name");
  }

  // Allocate the decoder context and set the options to enable error concealment
  codec_ctx_.reset(avcodec_alloc_context3(codec));
  if (!codec_ctx_) {
    throw Error("Decoder::Decoder(): Failed to allocate codec context");
  }
  codec_ctx_->workaround_bugs = FF_BUG_AUTODETECT;
  codec_ctx_->err_recognition = AV_EF_CRCCHECK;
  codec_ctx_->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;

  // Create a hardware acceleration context supported by the decoder.
  // If multiple hardware devices are supported, the first one is used.
  if (!codec_ctx_->hw_device_ctx) {
    codec_ctx_->hw_device_ctx = [codec]() {
      for (int i = 0;; ++i) {
        if (const AVCodecHWConfig *const hw_config = avcodec_get_hw_config(codec, i);
            hw_config && (hw_config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX)) {
          // Return the context of i-th hardware device supported by the decoder, if successful
          AVBufferRef *hw_device_ctx;
          if (av_hwdevice_ctx_create(&hw_device_ctx, hw_config->device_type, nullptr, nullptr, 0) ==
              0) {
            return hw_device_ctx;
          }
        } else if (!hw_config) {
          // Return nullptr if no more hardware devices are supported by the decoder
          return static_cast<AVBufferRef *>(nullptr);
        }
      }
    }();
  }

  // Open the decoder
  if (const int ret = avcodec_open2(codec_ctx_.get(), codec, nullptr); ret < 0) {
    throw Error("Decoder::Decoder(): Failed to open codec", ret);
  }
}

void Decoder::send_packet(const Packet &packet) {
  if (const int ret = avcodec_send_packet(codec_ctx_.get(), packet.get()); ret < 0) {
    throw Error("Decoder::send_packet(): Error sending packet for decoding", ret);
  }
}

bool Decoder::receive_frame(Frame *const frame) {
  frame->unref();
  if (const int ret = avcodec_receive_frame(codec_ctx_.get(), frame->get()); ret == 0) {
    return true;
  } else if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
    // The decoder is in an OK state, but no frame is currently available
    return false;
  } else {
    throw Error("Decoder::receive_frame(): Error during decoding", ret);
  }
}

std::string Decoder::codec_name() const { return avcodec_get_name(codec_ctx_->codec_id); }

std::string Decoder::hw_device_type() const {
  return codec_ctx_->hw_device_ctx
             ? av_hwdevice_get_type_name(
                   reinterpret_cast<AVHWDeviceContext *>(codec_ctx_->hw_device_ctx->data)->type)
             : "none";
}

void Decoder::free_context(AVCodecContext *codec_ctx) { avcodec_free_context(&codec_ctx); }

// ==============================================
// Parser - RAII wrapper for AVCodecParserContext
// ==============================================

Parser::Parser(const std::string &codec_name) : parser_ctx_(nullptr, &av_parser_close) {
  // Find the codec by name
  const AVCodec *const codec = avcodec_find_decoder_by_name(codec_name.c_str());
  if (!codec) {
    throw Error("Parser::Parser(): " + codec_name + " was not recognized as a codec name");
  }

  // Initialize the parser with the codec
  parser_ctx_.reset(av_parser_init(codec->id));
  if (!parser_ctx_) {
    throw Error("Parser::Parser(): Failed to initialize parser for codec (" + codec_name + ")");
  }
}

int Parser::parse(const BufferRef &buffer, Decoder *const decoder, Packet *const packet,
                  const std::int64_t pos) {
  // Parse the buffer starting from the pos-th byte
  std::uint8_t *packet_data;
  int packet_size;
  const int len =
      av_parser_parse2(parser_ctx_.get(), decoder->get(), &packet_data, &packet_size, buffer->data,
                       buffer.unpadded_size(), AV_NOPTS_VALUE, AV_NOPTS_VALUE, pos);

  // Set the packet content based on the parser's output
  packet->unref();
  if (packet_data && buffer->data + pos <= packet_data &&
      packet_data + packet_size <= buffer->data + buffer.unpadded_size()) {
    // If a packet is available and the data is within the input buffer,
    // refer to the entire buffer from the packet, and set the data to the packet
    packet->ref(buffer);
    (*packet)->data = packet_data;
    (*packet)->size = packet_size;
  } else if (packet_data) {
    // If a packet is available but the data is within the parser's internal buffer,
    // copy the data from the parser's internal buffer to a new buffer
    // to guarantee the lifetime of the data, and refer to it from the packet
    packet->ref(BufferRef(packet_data, packet_size));
  }

  return len;
}

// =======================================
// Converter - RAII wrapper for SwsContext
// =======================================

Converter::Converter(const std::string &dst_format_name)
    : sws_ctx_(nullptr, &sws_freeContext), ctx_width_(0), ctx_height_(0),
      ctx_src_format_(AV_PIX_FMT_NONE), ctx_dst_format_(av_get_pix_fmt(dst_format_name.c_str())) {
  if (ctx_dst_format_ == AV_PIX_FMT_NONE) {
    throw Error("Converter::Converter(): " + dst_format_name +
                " was not recognized as a destination pixel format");
  }
}

void Converter::convert(const Frame &src_frame, std::vector<std::uint8_t> *const dst_data) {
  // Find the deprecated pixel format of src frame to the corresponding non-deprecated one
  const AVPixelFormat src_frame_format = [](const AVPixelFormat maybe_deprecated_format) {
    switch (maybe_deprecated_format) {
    case AV_PIX_FMT_YUVJ420P:
      return AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_YUVJ411P:
      return AV_PIX_FMT_YUV411P;
    case AV_PIX_FMT_YUVJ422P:
      return AV_PIX_FMT_YUV422P;
    case AV_PIX_FMT_YUVJ440P:
      return AV_PIX_FMT_YUV440P;
    case AV_PIX_FMT_YUVJ444P:
      return AV_PIX_FMT_YUV444P;
    default:
      return maybe_deprecated_format;
    }
  }(static_cast<AVPixelFormat>(src_frame->format));

  // Create SwsContext if not created or the source frame has different size or format
  if (!sws_ctx_                             //
      || (ctx_width_ != src_frame->width)   //
      || (ctx_height_ != src_frame->height) //
      || (ctx_src_format_ != src_frame_format)) {
    sws_ctx_.reset(sws_getContext(
        // src description
        src_frame->width, src_frame->height, src_frame_format,
        // dst description (keep the same size)
        src_frame->width, src_frame->height, ctx_dst_format_,
        // options for scaling and filtering (won't be used)
        SWS_BILINEAR, nullptr, nullptr, nullptr));
    if (!sws_ctx_) {
      throw Error("Converter::convert(): Failed to create SwsContext");
    }
    ctx_width_ = src_frame->width;
    ctx_height_ = src_frame->height;
    ctx_src_format_ = src_frame_format;
  }

  // Get the layout of the destination image
  // - linesize: bytes per line for each plane
  std::array<int, 4> dst_linesize;
  if (const int ret = av_image_fill_linesizes(dst_linesize.data(), ctx_dst_format_, ctx_width_);
      ret < 0) {
    throw Error("Converter::convert(): Failed to get destination linesizes", ret);
  }
  // - plane size: bytes per plane
  std::array<std::size_t, 4> dst_plane_size;
  if (const int ret =
          av_image_fill_plane_sizes(dst_plane_size.data(), ctx_dst_format_, ctx_height_,
                                    std::array<std::ptrdiff_t, 4>{dst_linesize[0], dst_linesize[1],
                                                                  dst_linesize[2], dst_linesize[3]}
                                        .data());
      ret < 0) {
    throw Error("Converter::convert(): Failed to get destination plane sizes", ret);
  }
  // - data offset: bytes to the start of each plane
  std::array<std::ptrdiff_t, 4> dst_data_offset;
  std::partial_sum(dst_plane_size.begin(), dst_plane_size.end(), dst_data_offset.begin());

  // Resize the destination buffer to the sum of the plane sizes
  dst_data->resize(dst_data_offset[3]);

  // Convert the pixel data
  sws_scale(sws_ctx_.get(),
            // src description
            src_frame->data, src_frame->linesize, 0, src_frame->height,
            // dst description
            std::array<std::uint8_t *const, 4>{
                dst_data->data(), dst_data->data() + dst_data_offset[0],
                dst_data->data() + dst_data_offset[1], dst_data->data() + dst_data_offset[2]}
                .data(),
            dst_linesize.data());
}

std::string Converter::dst_format_name() const { return av_get_pix_fmt_name(ctx_dst_format_); }

} // namespace libav_cpp