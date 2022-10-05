#!/bin/bash

# Give the flyMS program access to modify the PRU
sudo setcap cap_sys_rawio+ep /home/debian/bin/flyMS

sudo systemctl restart flyMS
sudo systemctl restart PruManager
sudo systemctl restart flyMS_webserver
sudo systemctl restart mission_interface
