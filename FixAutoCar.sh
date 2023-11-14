
wget https://bootstrap.pypa.io/pip/3.6/get-pip.py

sudo -H python3 get-pip.py

sudo ln -s /usr/local/bin/pip3.6 /usr/bin/pip3 

sudo pip3 install setuptools --upgrade
sudo pip3 install python-can --force-reinstall
