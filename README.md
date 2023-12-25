### Fix zsh_history

```
chmod +x ./fix_zsh.sh
./fix_zsh.sh
```

### Fix I2C

```
first:
chmod +x ./fix_i2c.sh 
./fix_i2c.sh
next:
pip install python-can --force-reinstall
```

### Config connect to server

```
/auto_car/.env
```

The field SERVER_SOCKETIO and SERVER_RTMP in .env file: you must change the IP address to the server’s IP address.

The field ID in .env file: you must change the ID of robot. It is unique for each robot

### install environment

```
pip install -r requirements.txt
```

### check usb gps

```
ls /dev/ttyUSB*
/dev/ttyUSB1
sudo chmod +x 666 /dev/ttyUSB1
```

## ROS2 Eloquent

```
cd auto_car
```

### Build node ROS2

```
colcon build
```

### Setup env ROS2

```
source ./install/setup.zsh
# Different id for each car.
export ROS_DOMAIN_ID=1
```

### Check auto_car node

```
ros2 pkg list
ros2 pkg executables
```

### all-in-one:

```
ros2 launch auto_car auto_car.launch.py
```

### check log:

```
ros2 topic echo /rosout
```

### If want to debug, run each node:

```
ros2 run auto_car planning
ros2 run auto_car gps
ros2 run auto_car socketio
ros2 run auto_car controller
```
