#!/bin/bash

# Convert a sequence of images to videos using FFmpeg

# Convert 180-degree camera images to video
ffmpeg -framerate 22 -pattern_type glob -i 'jabra_180camera_image_raw_compressed/*.png' -c:v h264_nvenc -pix_fmt yuv420p 180cam.mp4

# Convert thermal camera images to video
ffmpeg -framerate 30 -pattern_type glob -i 'thermal_image_compressed/*.png' -c:v h264_nvenc -pix_fmt yuv420p thermal.mp4
