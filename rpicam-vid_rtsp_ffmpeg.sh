#!/bin/bash
rpicam-vid -t 0 -n --width 320 --height 240 --inline --codec libav --libav-format mpegts -o - | ffmpeg -i - -c:v copy -f rtsp rtsp://localhost:8554/mystream
