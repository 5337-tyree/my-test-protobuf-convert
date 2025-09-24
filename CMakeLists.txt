cmake_minimum_required(VERSION 3.15)
project(embed_protobuf_example CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


find_package(Protobuf REQUIRED)


message(STATUS "Using Protobuf ${Protobuf_VERSION}")


protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/gyroflow.proto
)


add_executable(embed_protobuf
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  src/embed_protobuf.cpp
)

include_directories(
  ${AVFORMAT_INCLUDE_DIRS}
  ${AVCODEC_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
)

link_directories(
  ${AVFORMAT_LIBRARY_DIRS}
  ${AVCODEC_LIBRARY_DIRS}
  ${AVUTIL_LIBRARY_DIRS}
)

find_package(absl REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)


target_link_libraries(embed_protobuf
  ${Protobuf_LIBRARIES}
  absl::log_internal_check_op
  absl::log_internal_message
  absl::log_severity
  absl::core_headers
  ${AVFORMAT_LIBRARIES}
  ${AVCODEC_LIBRARIES}
  ${AVUTIL_LIBRARIES}
)


target_include_directories(embed_protobuf PRIVATE
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)
