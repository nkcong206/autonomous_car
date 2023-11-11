module.exports = {
    apps: 
    [
        {
            name: 'socketioclient',
            interpreter: 'python3',
            script: 'client_stream.py',
            instances: 1,
            autorestart: true,
            watch: false,
            max_memory_restart: '1G',
            env: {
                NODE_ENV: 'production'
            }
        },
        {
            name: 'stream',
            script: 'ffmpeg',
            args: ['-f', 'v4l2', '-input_format', 'mjpeg', '-framerate', '10', '-video_size', '640x480', '-i', '/dev/video2', '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency', '-f', 'flv', 'rtmp://192.168.2.123:1935/live/test'],
            instances: 1,
            autorestart: true,
            watch: false,
            max_memory_restart: '1G',
            env: {
                NODE_ENV: 'production'
            }
        },
        {
            name: 'stream_gst',
            script: 'sh ./runstream.sh',
            args: [],
            instances: 1,
            autorestart: true,
            watch: false,
            max_memory_restart: '1G',
            env: {
                NODE_ENV: 'production'
            }
        }
    ]
};
