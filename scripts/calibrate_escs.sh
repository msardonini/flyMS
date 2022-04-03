#!/bin/bash

sudo systemctl stop flyMS
sudo systemctl stop pru_handler

sudo rc_calibrate_escs

sudo systemctl start pru_handler
sudo systemctl start flyMS
