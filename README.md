### Fix I2C
    chmod +x ./fix_i2c.sh 
    sh ./fix_i2c.sh
### Config connect to server
    /auto_car/auto_car/.env
The field SERVER_SOCKETIO and SERVER_RTMP in .env file: you must change the IP address to the server’s IP address.

The field ID in .env file: you must change the ID of robot. It is unique for each robot

### install environment

    pip install -r requirements.txt

## ROS2 Eloquent
    cd auto_car
### check usb gps
    ls /dev/ttyUSB*
### Build ROS2 node
    colcon build --packages-select auto_car
### Setup env ROS2
    source ./install/setup.zsh

### all-in-one
    ros2 launch auto_car auto_car.launch.py
### check log
    ros2 topic echo /rosout
    
### Run each node to debug:

    ros2 run auto_car planning
    ros2 run auto_car gps
    ros2 run auto_car socketio
    ros2 run auto_car controller