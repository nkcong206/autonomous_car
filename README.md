## Fix error
### Fix zsh_history

```
chmod +x ./fix_zsh.sh
./fix_zsh.sh
```
### Fix I2C
- First time:

```
chmod +x ./fix_i2c.sh 
./fix_i2c.sh
```
- Next time:

```
sudo pip install python-can --force-reinstall
```

## Run ROS2

```
cd auto_car
```
### Install environment

```
pip install -r requirements.txt
```
### Config connect to server
Edit file ```/.env```:
```
SERVER_SOCKETIO=http://192.168.200.184:5001
SERVER_RTMP=rtmp://192.168.200.184:1935
ID=robot1
NAME=1234
```
The field SERVER_SOCKETIO and SERVER_RTMP in .env file: you must change the IP address to the server’s IP address.

The field ID in .env file: you must change the ID of robot. It is unique for each robot.

### Build node ROS2

```
colcon build
```
### Setup env ROS2

```
source ./install/setup.zsh
# Different id for each car.
export ROS_DOMAIN_ID=id
```
### Check GPS

```
ls /dev/ttyUSB*
```
Output: ```/dev/ttyUSB1``` -> GPS has connected. 
### Check auto_car node

```
ros2 pkg list
```
or
```
ros2 pkg executables
```
### Debug
- Permission:

```
sudo chmod +x 666 /dev/ttyUSB1

```
- Run 4 commands with each terminal:

```
ros2 run auto_car gps
ros2 run auto_car socketio
ros2 run auto_car planning
ros2 run auto_car controller
```
To stop, press Ctrl+C and wait for the terminal to show "Node stopped!"
### All-in-One:
- Just run 1 command instead of running the commands as above:

```
ros2 launch auto_car auto_car.launch.py
```

- Show log in other terminal

```
ros2 topic echo /rosout
```
