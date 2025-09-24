# -*- coding: utf-8 -*-

import struct
from gyroflow_pb2 import Main, FrameMetadata, IMUData

gcsv_path = "HOVER_SPLASH_0025.gcsv"
output_path = "HOVER_SPLASH_0025.bin"
video_fps = 60.0 
imu_rate = 1000.0 
tscale = 1e-6
# ====================

def parse_gyroflow_log(path):
    """ 解析 Gyroflow IMU Log 格式 """
    imu_list = []
    with open(path, 'r') as f:
        lines = f.readlines()
    
    # skip no head date 
    data_start = 0
    for i, line in enumerate(lines):
        if line.strip().startswith("t,"):
            data_start = i + 1
            break

    for line in lines[data_start:]:
        parts = [p.strip() for p in line.strip().split(",")]
        if len(parts) < 7:
            continue  # skip invalid line
        try:
            imu = IMUData()
            imu.sample_timestamp_us = int(parts[0]) * tscale * 1e6
            imu.gyroscope_x = float(parts[1])
            imu.gyroscope_y = float(parts[2])
            imu.gyroscope_z = float(parts[3])
            imu.accelerometer_x = float(parts[4])
            imu.accelerometer_y = float(parts[5])
            imu.accelerometer_z = float(parts[6])
            imu_list.append(imu)
        except Exception as e:
            print("skip error line:", line)
            continue

    return imu_list

def build_main_proto(imu_data_list):
    samples_per_frame = int(imu_rate / video_fps)
    total_frames = len(imu_data_list) // samples_per_frame

    frames = []
    for i in range(total_frames):
        m = Main()
        m.magic_string = "GyroflowProtobuf"
        m.protocol_version = 1

        frame = m.frame
        frame.frame_number = i + 1
        frame_imus = imu_data_list[i * samples_per_frame : (i + 1) * samples_per_frame]
        if not frame_imus:
            continue

        frame.start_timestamp_us = frame_imus[0].sample_timestamp_us
        frame.end_timestamp_us   = frame_imus[-1].sample_timestamp_us
        frame.imu.extend(frame_imus)

        frames.append(m)
    return frames

def write_protobuf_file(frames, output_path):
    with open(output_path, 'wb') as f:
        for frame in frames:
            data = frame.SerializeToString()
            f.write(struct.pack("<I", len(data)))
            f.write(data)

if __name__ == "__main__":

    imu_data = parse_gyroflow_log(gcsv_path)


    proto_frames = build_main_proto(imu_data)


    write_protobuf_file(proto_frames, output_path)

    print("✅ done!")
