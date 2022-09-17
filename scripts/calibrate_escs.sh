#!/bin/bash

sudo systemctl stop flyMS
sudo systemctl stop PruManager

sudo rc_calibrate_escs

sudo systemctl start PruManager
sudo systemctl start flyMS
