rpicam-vid -t 0 -n --codec libav --libav-format mpegts -o - | cvlc -vvv stream:///dev/stdin --sout '#rtp{sdp=rtsp://192.168.0.31:8555/stream1}' 
