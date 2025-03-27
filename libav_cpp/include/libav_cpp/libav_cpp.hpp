#ifndef LIBAV_CPP_LIBAV_CPP_HPP
#define LIBAV_CPP_LIBAV_CPP_HPP

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

namespace libav_cpp {

// Error with an optional error number from libav
class Error : public std::runtime_error {
public:
  Error(const std::string &msg) : std::runtime_error(msg) {}
  Error(const std::string &msg, const int errnum)
      : std::runtime_error(msg + ": " + err2str(errnum)) {}

private:
  // ex. err2str(EINVAL) -> "Invalid argument"
  static std::string err2str(const int errnum);
};

// RAII wrapper for AVBufferRef
class BufferRef {
public:
  BufferRef(const std::size_t unpadded_size);
  BufferRef(const std::uint8_t *const data, const std::size_t unpadded_size)
      : BufferRef(unpadded_size) {
    std::copy(data, data + unpadded_size, buf_->data);
  }
  BufferRef(const BufferRef &buf);

  AVBufferRef *get() { return buf_.get(); }
  const AVBufferRef *get() const { return buf_.get(); }
  AVBufferRef *operator->() { return buf_.operator->(); }
  const AVBufferRef *operator->() const { return buf_.operator->(); }
  std::size_t padded_size() const { return buf_->size; }
  std::size_t unpadded_size() const { return buf_->size - AV_INPUT_BUFFER_PADDING_SIZE; }

private:
  static void unref_buffer(AVBufferRef *buf);

private:
  std::unique_ptr<AVBufferRef, decltype(&unref_buffer)> buf_;
};

// RAII wrapper for AVPacket
class Packet {
public:
  // Allocate the packet and default the fields
  Packet();

  void ref(const BufferRef &buf);

  // Detach the packet from the data buffer if any, and default the rest of the fields
  void unref();

  AVPacket *get() { return packet_.get(); }
  const AVPacket *get() const { return packet_.get(); }
  AVPacket *operator->() { return packet_.operator->(); }
  const AVPacket *operator->() const { return packet_.operator->(); }

private:
  static void free_packet(AVPacket *packet);

private:
  std::unique_ptr<AVPacket, decltype(&free_packet)> packet_;
};

// RAII wrapper for AVFrame
class Frame {
public:
  // Allocate the frame and default the fields
  Frame();

  // Detach the frame from the data buffers if any, and default the rest of the fields
  void unref();

  // True if the frame is on a hardware memory, which is not accessible by the CPU
  bool is_hw_frame() const { return frame_->hw_frames_ctx != nullptr; }

  // Copy the frame to the CPU-accessible memory
  void transfer_data(Frame *const dst) const;

  AVFrame *get() { return frame_.get(); }
  const AVFrame *get() const { return frame_.get(); }
  AVFrame *operator->() { return frame_.operator->(); }
  const AVFrame *operator->() const { return frame_.operator->(); }

private:
  static void free_frame(AVFrame *frame);

private:
  std::unique_ptr<AVFrame, decltype(&free_frame)> frame_;
};

// RAII wrapper for input decice (a.k.a. AVFormatContext)
class Input {
public:
  // Open the input device by avformat_open_input() with the given URL, format and options
  // and find the best video stream
  Input(const std::string &url, const std::string &format_name,
        const std::map<std::string, std::string> &option_map);

  // Read the next frame from the video stream of interest
  void read_frame(Packet *const packet);

  AVFormatContext *get() { return format_ctx_.get(); }
  const AVFormatContext *get() const { return format_ctx_.get(); }
  AVFormatContext *operator->() { return format_ctx_.operator->(); }
  const AVFormatContext *operator->() const { return format_ctx_.operator->(); }
  int stream_id() const { return stream_id_; }

private:
  static void close_input(AVFormatContext *format_ctx);

private:
  std::unique_ptr<AVFormatContext, decltype(&close_input)> format_ctx_;
  int stream_id_;
};

// RAII wrapper for decoder (a.k.a AVCodecContext)
class Decoder {
public:
  // Initialize the decoder with the given codec name
  Decoder(const std::string &codec_name);

  // Send a cmpressed packet to the decoder
  void send_packet(const Packet &packet);

  // True if a decoded frame is received, false if no more frames are available
  bool receive_frame(Frame *const frame);

  AVCodecContext *get() { return codec_ctx_.get(); }
  const AVCodecContext *get() const { return codec_ctx_.get(); }
  AVCodecContext *operator->() { return codec_ctx_.operator->(); }
  const AVCodecContext *operator->() const { return codec_ctx_.operator->(); }

  std::string codec_name() const;
  std::string hw_device_type() const;

private:
  static void free_context(AVCodecContext *codec_ctx);

private:
  std::unique_ptr<AVCodecContext, decltype(&free_context)> codec_ctx_;
};

// RAII wrapper for AVParserContext
class Parser {
public:
  // Initialize the parser with the given codec ID
  Parser(const std::string &codec_name);

  // Parse the buffer starting from the pos-th byte, and store the data of one frame in the packet.
  // The data in the packet is managed by reference counting, so its lifetime is guaranteed.
  // Return the number of bytes successfully parsed.
  int parse(const BufferRef &buffer, Decoder *const decoder, Packet *const packet,
            const std::int64_t pos);

  AVCodecParserContext *get() { return parser_ctx_.get(); }
  const AVCodecParserContext *get() const { return parser_ctx_.get(); }
  AVCodecParserContext *operator->() { return parser_ctx_.operator->(); }
  const AVCodecParserContext *operator->() const { return parser_ctx_.operator->(); }

private:
  std::unique_ptr<AVCodecParserContext, decltype(&av_parser_close)> parser_ctx_;
};

// RAII wrapper for SwsContext
class Converter {
public:
  // Initialize the converter with the destination pixel format name
  Converter(const std::string &dst_format_name);

  // Convert the source frame to the destination pixel format
  void convert(const Frame &src_frame, std::vector<std::uint8_t> *const dst_data);

  std::string dst_format_name() const;

private:
  std::unique_ptr<SwsContext, decltype(&sws_freeContext)> sws_ctx_;
  int ctx_width_, ctx_height_;
  AVPixelFormat ctx_src_format_;
  const AVPixelFormat ctx_dst_format_;
};

} // namespace libav_cpp

#endif