#!/bin/sh

sudo cp services/flyMS.service /etc/systemd/system/
sudo cp services/flyMS_webserver.service /etc/systemd/system/
sudo cp services/pru_handler.service /etc/systemd/system/
sudo cp services/redis.service /etc/systemd/system/
sudo cp ./debian_sudoers /etc/sudoers.d/debian

sudo systemctl daemon-reload
sudo systemctl enable flyMS
sudo systemctl enable flyMS_webserver
sudo systemctl enable pru_handler
sudo systemctl enable redis


sudo pip3 install -r requirements.txt

curl -fsSL get.docker.com -o get-docker.sh && sh get-docker.sh
sudo usermod -aG docker debian


# Enable high priority threads, which is used to get near real-time performance
sudo echo "debian hard rtprio 99" >> /etc/security/limits.conf
sudo echo "debian soft rtprio 99" >> /etc/security/limits.conf
