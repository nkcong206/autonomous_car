module.exports = {
    apps: 
    [
        {
            name: 'socketioclient',
            interpreter: 'python3',
            script: 'socketio_node_manual.py',
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
