### Client controller will run on jeson nano 

### 1 Install pm2 on jetson nano 
sudo apt update
sudo apt install nodejs
node -v
sudo apt install npm
npm install pm2 -g

### 2 Run client controller 

the field SERVER_SOCKETIO and SERVER_RTMP in .env file . You must change the IP address to the server’s IP address. 
the field ID in .env file . You must change the ID of robot. It is unique for each robot 
After that, you run this command 

pm2 start ecosystem.config.js
pm2 save