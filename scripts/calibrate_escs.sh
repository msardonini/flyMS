#!/bin/bash

sudo systemctl stop flyMS

sudo rc_calibrate_escs

sudo systemctl start flyMS
