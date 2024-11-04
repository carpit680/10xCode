#!/bin/bash -e

USER_ID=$(id -u)
GROUP_ID=$(id -g)
USER=${USER}
GROUP=${GROUP}
PASSWD=${PASSWD}

# Start DBus without systemd
sudo /etc/init.d/dbus start

export DISPLAY=":0"

# Set login user name
USER=$(whoami)
echo "USER: $USER"

# Set login password
echo "PASSWD: $PASSWD"
echo ${USER}:${PASSWD} | sudo chpasswd

# SSH start
sudo service ssh start

dbus-launch fcitx &

sudo chsh -s /usr/bin/zsh $USER
export SHELL=/usr/bin/zsh
