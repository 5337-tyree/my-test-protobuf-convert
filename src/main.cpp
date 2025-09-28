#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <cassert>

#include "gyroflow.pb.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libavutil/mem.h>
}

// IMU 样本结构
struct ImuSample {
    double timestamp_us;
    float gx, gy, gz;
    float ax, ay, az;
};

std::string readFileToString(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// 解析 GCSV 日志到样本列表
bool parse_gcsv(const char *gcsv_path, std::vector<ImuSample> &out_samples) {
    std::ifstream ifs(gcsv_path);
    if (!ifs.is_open()) {
        fprintf(stderr, "Failed to open gcsv file: %s\n", gcsv_path);
        return false;
    }
    std::string line;
    bool in_data = false;
    while (std::getline(ifs, line)) {
        if (!in_data) {
            // 找到表头 “t,gx,gy,gz,ax,ay,az”
            if (line.rfind("t,gx,gy,gz,ax,ay,az", 0) == 0) {
                in_data = true;
            }
            continue;
        }
        if (line.empty()) continue;
        std::istringstream ss(line);
        ImuSample s;
        char comma;
        ss >> s.timestamp_us;
        ss >> comma >> s.gx;
        ss >> comma >> s.gy;
        ss >> comma >> s.gz;
        ss >> comma >> s.ax;
        ss >> comma >> s.ay;
        ss >> comma >> s.az;

        //fprintf(stderr, " timestamp: %f", s.timestamp_us);
        if (!ss.fail()) {
            out_samples.push_back(s);
        } else {
            fprintf(stderr, "Warning: parse failed line: %s\n", line.c_str());
        }
    }
    return true;
}

// FFmpeg 错误输出
void ff_err(const char *msg, int err) {
    char buf[256];
    av_strerror(err, buf, sizeof(buf));
    fprintf(stderr, "%s: %s (err=%d)\n", msg, buf, err);
}

// 为输出上下文添加一个 metadata stream（用于 gyroflow protobuf）
AVStream *add_metadata_stream(AVFormatContext *ofmt_ctx) {
    AVStream *st = avformat_new_stream(ofmt_ctx, nullptr);
    if (!st) {
        fprintf(stderr, "Could not create metadata stream\n");
        return nullptr;
    }
    st->id = ofmt_ctx->nb_streams - 1;
    st->codecpar->codec_type = AVMEDIA_TYPE_DATA;
    st->codecpar->codec_id = AV_CODEC_ID_BIN_DATA;
    //st->codecpar->codec_tag = MKTAG('g', 'f', 'd', 't');  // Gyroflow data tag
    st->time_base = AVRational{1, 1000000};  // 微秒单位
    av_dict_set(&st->metadata, "handler_name", "GyroflowTelemetry", 0);
    return st;
}

// 写入 protobuf packet 到 metadata stream
int write_meta_packet(AVFormatContext *ofmt_ctx, AVStream *meta_st,
                      const std::string &buf, int64_t ts_us) {
    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.stream_index = meta_st->index;
    pkt.data = (uint8_t *)av_malloc(buf.size());
    if (!pkt.data) {
        fprintf(stderr, "av_malloc failed for meta packet\n");
        return AVERROR(ENOMEM);
    }
    memcpy(pkt.data, buf.data(), buf.size());
    pkt.size = buf.size();

    pkt.pts = ts_us;
    pkt.dts = pkt.pts;
    pkt.duration = 0;

    int ret = av_interleaved_write_frame(ofmt_ctx, &pkt);
    if (ret < 0) {
        ff_err("write_meta_packet failed", ret);
    }
    av_free(pkt.data);
    return ret;
}

// 填充 gyroflow::Main 消息（包括 header / frame / IMU 样本）
void fill_gyroflow_message(gyroflow::Main &msg,
                           bool is_first,
                           int frame_idx,
                           int64_t frame_start_us,
                           int64_t frame_end_us,
                           const std::vector<ImuSample> &imu_samples) {
    msg.clear_header();
    msg.clear_frame();
    msg.set_magic_string("GyroflowProtobuf");
    msg.set_protocol_version(1);

    if (is_first) {

        std::string jsonStr = readFileToString("/home/zerozero/tyree/gyroflow_convert/HOVER_SPLASH_0025.json");
        if (jsonStr.empty()) {
            std::cout << "Lens Profile JSON error:\n" << std::endl;
            return ;
        }
        //std::cout << "Lens Profile JSON:\n" << jsonStr << std::endl;
        gyroflow::Header *hdr = msg.mutable_header();
        // Camera metadata（你根据实际填）
        auto *cam = hdr->mutable_camera();
        cam->set_camera_brand("ZEROZERO");
        cam->set_camera_model("X1promax");
        cam->set_firmware_version("FIRMWARE_0.1.0");
        cam->set_lens_brand("ZEROZERO");
        cam->set_lens_model("720p_4by3");
        cam->set_sensor_pixel_width(1280);
        cam->set_sensor_pixel_height(960);
        //cam->set_pixel_pitch_nm(1);
        // 其它可选字段略
        cam->set_crop_factor(1.0f);
        cam->set_lens_profile(jsonStr);
        gyroflow::Header::ClipMetadata *clip = hdr->mutable_clip();
        clip->set_frame_width(3840);
        clip->set_frame_height(2880);
        clip->set_record_frame_rate(60.0f);
        clip->set_sensor_frame_rate(60.0f);
        clip->set_file_frame_rate(60.0f);
        clip->set_rotation_degrees(0);
        clip->set_imu_sample_rate(1000);
        clip->set_pixel_aspect_ratio(1.0f);
        clip->set_frame_readout_time_us(8.333f);
        clip->set_frame_readout_direction(gyroflow::Header::ClipMetadata::TopToBottom);
    }

    gyroflow::FrameMetadata *fm = msg.mutable_frame();
    fm->set_frame_number(frame_idx + 1);
    fm->set_start_timestamp_us((double)frame_start_us);
    fm->set_end_timestamp_us((double)frame_end_us);

    for (const auto &s : imu_samples) {
        //fprintf(stderr, "start us %f, end us %f, real %f\n", frame_start_us, frame_end_us, s.timestamp_us);
        if (s.timestamp_us >= frame_start_us && s.timestamp_us <= frame_end_us) {            //fprintf(stderr, "tyree in  frame count :%d\n", frame_idx);
            gyroflow::IMUData *d = fm->add_imu();
            d->set_sample_timestamp_us(s.timestamp_us);


            d->set_gyroscope_x(s.gx);
            d->set_gyroscope_y(s.gy);
            d->set_gyroscope_z(s.gz);
            d->set_accelerometer_x(s.ax);
            d->set_accelerometer_y(s.ay);
            d->set_accelerometer_z(s.az);
        }
    }
}

// 给定帧索引，计算帧在输出中的 start / end 微秒时间（你可以改为合适映射）  
void get_frame_time_bounds(int frame_idx, int64_t &out_start_us, int64_t &out_end_us) {
    int64_t first_ts = 376959328;
    double interval_us = 1000000.0 / 60.0;
    out_start_us = 376959328;// + (int64_t)(frame_idx * interval_us);
    out_end_us   = 388548834;//376959328 + (int64_t)((frame_idx + 1) * interval_us);
}

int main(int argc, char *argv[]) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s input_video.mp4 imu_log.gcsv output_with_meta.mp4\n", argv[0]);
        return 1;
    }
    const char *in_video = argv[1];
    const char *gcsv = argv[2];
    const char *out_mp4 = argv[3];

    std::vector<ImuSample> imu_samples;
    if (!parse_gcsv(gcsv, imu_samples)) {
        fprintf(stderr, "parse_gcsv failed\n");
        return 1;
    }
    fprintf(stderr, "Read %zu IMU samples\n", imu_samples.size());

    avformat_network_init();

    AVFormatContext *ifmt_ctx = nullptr;
    int ret = avformat_open_input(&ifmt_ctx, in_video, nullptr, nullptr);
    if (ret < 0) {
        ff_err("open input failed", ret);
        return 1;
    }
    ret = avformat_find_stream_info(ifmt_ctx, nullptr);
    if (ret < 0) {
        ff_err("find stream info failed", ret);
        return 1;
    }

    int vid_stream_idx = -1;
    for (unsigned i = 0; i < ifmt_ctx->nb_streams; i++) {
        if (ifmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            vid_stream_idx = i;
            break;
        }
    }
    if (vid_stream_idx < 0) {
        fprintf(stderr, "No video stream in input\n");
        return 1;
    }

    AVFormatContext *ofmt_ctx = nullptr;
    ret = avformat_alloc_output_context2(&ofmt_ctx, nullptr, nullptr, out_mp4);
    if (ret < 0 || !ofmt_ctx) {
        ff_err("alloc_output failed", ret);
        return 1;
    }

    AVStream *out_vid_st = avformat_new_stream(ofmt_ctx, nullptr);
    if (!out_vid_st) {
        fprintf(stderr, "Failed creating output video stream\n");
        return 1;
    }
    ret = avcodec_parameters_copy(out_vid_st->codecpar, ifmt_ctx->streams[vid_stream_idx]->codecpar);
    if (ret < 0) {
        ff_err("copy codecpar failed", ret);
        return 1;
    }
    out_vid_st->codecpar->codec_tag = 0;
    out_vid_st->time_base = ifmt_ctx->streams[vid_stream_idx]->time_base;

    AVStream *meta_st = add_metadata_stream(ofmt_ctx);
    if (!meta_st) {
        fprintf(stderr, "Failed to create metadata stream\n");
        return 1;
    }

    if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        ret = avio_open(&ofmt_ctx->pb, out_mp4, AVIO_FLAG_WRITE);
        if (ret < 0) {
            ff_err("avio_open output failed", ret);
            return 1;
        }
    }

    ret = avformat_write_header(ofmt_ctx, nullptr);
    if (ret < 0) {
        ff_err("write_header failed", ret);
        return 1;
    }

    AVPacket pkt;
    int frame_count = 0;
    while (true) {
        ret = av_read_frame(ifmt_ctx, &pkt);
        if (ret < 0) break;
        if (pkt.stream_index == vid_stream_idx) {
            // 写视频包（stream copy）
            pkt.stream_index = out_vid_st->index;
            ret = av_interleaved_write_frame(ofmt_ctx, &pkt);
            if (ret < 0) {
                ff_err("write video packet failed", ret);
                break;
            }

            int64_t start_us, end_us;
            
            get_frame_time_bounds(frame_count, start_us, end_us);
            
            gyroflow::Main msg;
            bool is_first = (frame_count == 0);
            fill_gyroflow_message(msg, is_first, frame_count, start_us, end_us, imu_samples);

            std::string buf;
            if (!msg.SerializeToString(&buf)) {
                fprintf(stderr, "SerializeToString failed frame %d\n", frame_count);
            } else {
                ret = write_meta_packet(ofmt_ctx, meta_st, buf, start_us);
                if (ret < 0) {
                    fprintf(stderr, "write_meta_packet failed frame %d\n", frame_count);
                }
            }

            frame_count++;
        }
        av_packet_unref(&pkt);
    }

    av_write_trailer(ofmt_ctx);

    if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        avio_closep(&ofmt_ctx->pb);
    }
    avformat_free_context(ofmt_ctx);
    avformat_close_input(&ifmt_ctx);
    avformat_network_deinit();

    fprintf(stderr, "Done. Frames: %d, IMU samples: %zu\n", frame_count, imu_samples.size());
    return 0;
}
