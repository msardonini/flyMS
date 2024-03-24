#!/bin/sh

sudo rm /etc/systemd/system/flyMS.service
sudo rm /etc/systemd/system/flyMS_webserver.service
sudo rm /etc/systemd/system/mission_interface.service

sudo systemctl disable flyMS
sudo systemctl disable flyMS_webserver
sudo systemctl disable mission_interface
sudo systemctl daemon-reload
