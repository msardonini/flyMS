#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

pushd $SCRIPT_DIR

# Enable the services needed for flyMS
sudo cp services/flyMS.service /etc/systemd/system/
sudo cp services/flyMS_webserver.service /etc/systemd/system/
sudo cp services/pru_handler.service /etc/systemd/system/
sudo cp services/redis.service /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable flyMS
sudo systemctl enable flyMS_webserver
sudo systemctl enable pru_handler
sudo systemctl enable redis

# Enable certain commands to run without sudo password
sudo cp ./debian_sudoers /etc/sudoers.d/debian

# Install python dependencies
sudo pip3 install -r requirements.txt

# # Install docker dependencies
# curl -fsSL get.docker.com -o get-docker.sh && sh get-docker.sh
# sudo usermod -aG docker debian

# Enable high priority threads, which is used to get near real-time performance. Only do this if the system is not
# already configured to do so
grep -qxF "debian hard rtprio 99" /etc/security/limits.conf || sudo echo "debian hard rtprio 99" >> \
  /etc/security/limits.conf
grep -qxF "debian soft rtprio 99" /etc/security/limits.conf || sudo echo "debian soft rtprio 99" >> \
  /etc/security/limits.conf

popd
