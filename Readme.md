# test project

## protoc version: 25.3

## file summary

    proto/                 -- source code from telemetry-parser

    src/embed_protobuf.cpp -- key code binary file emebed into source video file

    build.sh               -- compile script

    CMakeList.txt          -- building file on Linux

    gcsv_to_protobuf.py    -- script that convert gcsv to protobuf format

    HOVER_SPLASH_0025.mp4  -- source video file

    HOVER_SPLASH_0025.gcsv -- imu data, gcsv format

    HOVER_SPLASH_0025.json -- lens profile

    HOVER_SPLASH_0025.bin  -- binary file, protobuf format

    output.mov             -- output video file

## compile cmd

    ./build.sh 

## execute cmd

    ./build/embed_protobuf HOVER_SPLASH_0025.mp4 HOVER_SPLASH_0025.bin output.mov
