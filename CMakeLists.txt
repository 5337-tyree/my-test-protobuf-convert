cmake_minimum_required(VERSION 3.10)
project(gyroflow_mp4_demo LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 Protobuf
find_package(Protobuf REQUIRED)
message(STATUS "Protobuf version: ${Protobuf_VERSION}")

# 查找 FFmpeg 库（libavformat, libavcodec, libavutil）
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)

include_directories(
  ${Protobuf_INCLUDE_DIRS}
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
)

link_directories(
  ${AVFORMAT_LIBRARY_DIRS}
  ${AVCODEC_LIBRARY_DIRS}
  ${AVUTIL_LIBRARY_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
)

add_executable(gyroflow_mp4_demo
  src/main.cpp
  src/gyroflow.pb.cc
)

target_link_libraries(gyroflow_mp4_demo
  ${Protobuf_LIBRARIES}
  ${AVFORMAT_LIBRARIES}
  ${AVCODEC_LIBRARIES}
  ${AVUTIL_LIBRARIES}
  absl_log_internal_check_op
  absl_log_internal_message
  absl_strings
  absl_base
)
