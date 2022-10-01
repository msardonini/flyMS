#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

pushd $SCRIPT_DIR

# Enable the services needed for flyMS
sudo cp services/flyMS.service /etc/systemd/system/
sudo cp services/flyMS_webserver.service /etc/systemd/system/
sudo cp services/PruManager.service /etc/systemd/system/
sudo cp services/mission_interface.service /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable flyMS
sudo systemctl enable flyMS_webserver
sudo systemctl enable PruManager
sudo systemctl enable mission_interface

# Enable certain commands to run without sudo password
sudo cp ./debian_sudoers /etc/sudoers.d/debian

# Install python dependencies
sudo pip3 install -r requirements.txt

# Install the redis server
sudo apt-get install -y redis-server

# Enable high priority threads, which is used to get near real-time performance. Only do this if the system is not
# already configured to do so
grep -qxF "debian hard rtprio 99" /etc/security/limits.conf || sudo echo "debian hard rtprio 99" >> \
  /etc/security/limits.conf
grep -qxF "debian soft rtprio 99" /etc/security/limits.conf || sudo echo "debian soft rtprio 99" >> \
  /etc/security/limits.conf

popd

echo ""
echo "Please reboot the system to complete the installation!"
