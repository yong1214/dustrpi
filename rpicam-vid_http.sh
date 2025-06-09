rpicam-vid -t 0 -n --codec libav --libav-format mpegts -o - | cvlc -vvv stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst=:8160}'
