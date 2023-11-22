#!/bin/bash
cd ~
if [ ! -f "get-pip.py" ]; then
    wget https://bootstrap.pypa.io/pip/3.6/get-pip.py
fi

sudo -H python3 get-pip.py

if [ ! -f "/usr/bin/pip3" ]; then
    sudo ln -s /usr/local/bin/pip3.6 /usr/bin/pip3
fi

sudo pip3 install setuptools --upgrade
sudo pip3 install python-can --force-reinstall
