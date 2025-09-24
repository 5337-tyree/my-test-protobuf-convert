#include <iostream>
#include <fstream>
#include <vector>
#include <stdint.h>
#include <memory>
#include "gyroflow.pb.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/timestamp.h>
}

#include "gyroflow.pb.h"

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " input.mp4 imu.protobuf.bin output.mp4\n";
        return 1;
    }
    const char* input_video = argv[1];
    const char* bin_path     = argv[2];
    const char* output_video = argv[3];

    avformat_network_init();

    AVFormatContext* in_fmt_ctx = nullptr;
    if (avformat_open_input(&in_fmt_ctx, input_video, nullptr, nullptr) < 0) {
        std::cerr << "Error: could not open input video file: " << input_video << "\n";
        return 1;
    }
    if (avformat_find_stream_info(in_fmt_ctx, nullptr) < 0) {
        std::cerr << "Error: could not find stream info\n";
        avformat_close_input(&in_fmt_ctx);
        return 1;
    }

    int video_stream_index = -1;
    for (unsigned i = 0; i < in_fmt_ctx->nb_streams; i++) {
        if (in_fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index = i;
            break;
        }
    }
    if (video_stream_index < 0) {
        std::cerr << "Error: no video stream found in input\n";
        avformat_close_input(&in_fmt_ctx);
        return 1;
    }


    std::ifstream binfile(bin_path, std::ios::binary);
    if (!binfile) {
        std::cerr << "Error: cannot open bin file: " << bin_path << "\n";
        avformat_close_input(&in_fmt_ctx);
        return 1;
    }


    AVFormatContext* out_fmt_ctx = nullptr;
    avformat_alloc_output_context2(&out_fmt_ctx, nullptr, "mov", output_video);
    if (!out_fmt_ctx) {
        std::cerr << "Error: could not allocate output context\n";
        avformat_close_input(&in_fmt_ctx);
        return 1;
    }

    AVStream* in_video_stream = in_fmt_ctx->streams[video_stream_index];
    AVStream* out_video_stream = avformat_new_stream(out_fmt_ctx, nullptr);
    if (!out_video_stream) {
        std::cerr << "Error: could not create output video stream\n";
        avformat_close_input(&in_fmt_ctx);
        avformat_free_context(out_fmt_ctx);
        return 1;
    }
    avcodec_parameters_copy(out_video_stream->codecpar, in_video_stream->codecpar);
    out_video_stream->time_base = in_video_stream->time_base;

    AVStream* proto_stream = avformat_new_stream(out_fmt_ctx, nullptr);
    if (!proto_stream) {
        std::cerr << "Error: could not create protobuf data stream\n";
        avformat_close_input(&in_fmt_ctx);
        avformat_free_context(out_fmt_ctx);
        return 1;
    }
    proto_stream->codecpar->codec_type = AVMEDIA_TYPE_DATA;
    proto_stream->codecpar->codec_id = AV_CODEC_ID_NONE;
    proto_stream->codecpar->codec_tag = MKTAG('g', 'f', 'd', 't');  // Gyroflow data tag

    proto_stream->time_base = AVRational{1, 1000000};


    if (!(out_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        if (avio_open(&out_fmt_ctx->pb, output_video, AVIO_FLAG_WRITE) < 0) {
            std::cerr << "Error: could not open output file: " << output_video << "\n";
            avformat_close_input(&in_fmt_ctx);
            avformat_free_context(out_fmt_ctx);
            return 1;
        }
    }


    if (avformat_write_header(out_fmt_ctx, nullptr) < 0) {
        std::cerr << "Error: write header failed\n";
        avformat_close_input(&in_fmt_ctx);
        if (!(out_fmt_ctx->oformat->flags & AVFMT_NOFILE))
            avio_closep(&out_fmt_ctx->pb);
        avformat_free_context(out_fmt_ctx);
        return 1;
    }


    AVPacket pkt;
    av_init_packet(&pkt);

    int64_t proto_pts = 0;

    while (true) {
        uint32_t len = 0;
        if (!binfile.read(reinterpret_cast<char*>(&len), sizeof(len))) {

            break;
        }
        std::vector<char> buf;
        buf.resize(len);
        if (!binfile.read(buf.data(), len)) {
            std::cerr << "Warning: protobuf message shorter than expected\n";
            break;
        }

        // Deserialize protobuf to get timestamp
        gyroflow::Main m;
        if (!m.ParseFromArray(buf.data(), len)) {
            std::cerr << "Warning: failed to parse protobuf message at proto_pts=" << proto_pts << "\n";
   
        }


        if (av_read_frame(in_fmt_ctx, &pkt) < 0) {
            break;
        }

        if (pkt.stream_index == video_stream_index) {
            pkt.stream_index = out_video_stream->index;
            av_interleaved_write_frame(out_fmt_ctx, &pkt);
        }
        av_packet_unref(&pkt);

        AVPacket proto_pkt;
        av_init_packet(&proto_pkt);

        proto_pkt.data = reinterpret_cast<uint8_t*>(buf.data());
        proto_pkt.size = buf.size();
        proto_pkt.stream_index = proto_stream->index;

  
        double ts_us = m.frame().start_timestamp_us();
        int64_t pts = (int64_t)(ts_us * proto_stream->time_base.den / (proto_stream->time_base.num * 1000000.0));
        if (pts < 0) pts = proto_pts;
        proto_pkt.pts = pts;
        proto_pkt.dts = pts;
        proto_pkt.duration = 1; 
        proto_pkt.flags |= AV_PKT_FLAG_KEY;

        av_interleaved_write_frame(out_fmt_ctx, &proto_pkt);

        proto_pts = pts + 1;
    }


    av_write_trailer(out_fmt_ctx);


    avformat_close_input(&in_fmt_ctx);
    if (!(out_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        avio_closep(&out_fmt_ctx->pb);
    }
    avformat_free_context(out_fmt_ctx);

    std::cout << "Finished embedding protobuf into " << output_video << "\n";
    return 0;
}
