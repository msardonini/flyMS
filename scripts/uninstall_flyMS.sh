#!/bin/sh

sudo rm /etc/systemd/system/flyMS.service
sudo rm /etc/systemd/system/flyMS_webserver.service
sudo rm /etc/systemd/system/PruManager.service

sudo systemctl disable flyMS
sudo systemctl disable flyMS_webserver
sudo systemctl disable pruHandler
sudo systemctl daemon-reload
