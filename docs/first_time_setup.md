# First Time Setup

There are a few first time setup tasks that need to happen before use.

1. Flash The beagleboard to the latest image. At the time of this writing, Debian Buster is used. The headless
version is recommended (without graphical desktop) https://beagleboard.org/latest-images

2. Build and deploy flyMS to beagleboard. [See Build section](#build-environment)

3. Install services: After flyMS is built and deployed to the beagleboard, run:
```
sudo ~/bin/install_flyMS.sh
```

4. Enable high priority threads. In lieu of an RTOS, we use high priority threads. These need to enabled for before
used. This is done by appending the following lines to the file `/etc/security/limits.conf`.
For an example user 'debian':

```
debian hard rtprio 99
debian soft rtprio 99
```

5. Install the required python dependencies on the beaglebone.
```
sudo pip3 install pyyaml flask_restful
```

6. Make sure the beaglebone's debian packages are up to date
```
sudo apt update
sudo apt upgrade
```

7. Calibrate all Sensors

```
rc_calibrate_gyro
rc_calibrate_escs
rc_calibrate_accel
rc_calibrate_mag
```
