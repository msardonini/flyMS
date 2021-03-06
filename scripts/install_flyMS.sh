#!/bin/sh

sudo cp services/flyMS.service /etc/systemd/system/
sudo cp services/flyMS_webserver.service /etc/systemd/system/
sudo cp services/pru_handler.service /etc/systemd/system/
sudo cp ./debian_sudoers /etc/sudoers.d/debian

sudo systemctl daemon-reload
sudo systemctl enable flyMS
sudo systemctl enable flyMS_webserver
sudo systemctl enable pru_handler


sudo pip3 install -r requirements.txt
