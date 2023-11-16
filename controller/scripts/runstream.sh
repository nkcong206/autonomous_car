server_rtmp="1"
robot_id="1"
while IFS='=' read -r key value; do
    # echo "Key: $key, Value: $value"
    if [ $key = "SERVER_RTMP" ]; then
        server_rtmp=$value
    fi
    if [ $key = "ID" ]; then
        robot_id=$value
    fi
done < ../config/.env
echo $server_rtmp
echo $robot_id
gst-launch-1.0  nvarguscamerasrc ! 'video/x-raw(memory:NVMM),width=(int)640,height=(int)480,framerate=15/1' ! nvvidconv ! 'video/x-raw,format=I420' ! omxh264enc control-rate=2 bitrate=4000000 ! 'video/x-h264,stream-format=byte-stream' ! h264parse ! flvmux streamable=true name=mux ! rtmpsink location="$server_rtmp/live/$robot_id"