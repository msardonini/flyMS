# First Time Setup

## Required Hardware

1. [Beaglebone Blue](https://beagleboard.org/blue), or Beaglebone Black equipped with the
[RoboticsCape](https://www.newark.com/element14/bb-cape-robotics/robotics-cape-9-18-vdc/dp/95Y0637).
2. A Linux PC
3. A Spektrum Remote Controller (DX5+) with the
[satellite receiver](https://www.spektrumrc.com/Products/Default.aspx?ProdID=SPM9645)
4. Quadrotor hardware: Frame, Motors, ESCs, Battery
5. (Optional) A smartphone, for rapid PID tuning during outdoor testing

## Initial Software Setup

These are the steps required to get the software on the beaglebone running for the first time. These steps will only
need to be executed once. They are intended to be executed in order.

### Flash The Beagleboard
Flash the beaglebone to the [latest image](https://beagleboard.org/latest-images). At the time of this writing,
Debian Buster (version 2020-04-06) is used. The headless version is recommended (without graphical desktop)

### Get an SSH Sesssion
When plugged into your linux PC via USB, the beaglebone should automatically establish a network connection.
You can access it from either `192.168.6.2` or `192.168.7.2`. To determine which address is active, examine the
output of:

```bash
user@linuxPC:~$ ifconfig
```

> **_NOTE:_**  In most cases, both addresses are initialized

Then, to connect to the beaglebone, run:

```bash
user@linuxPC:~$ ssh debian@<address>
```

This project assumes that the default `debian` user is used. The default password is `temppwd`.

> **_NOTE:_**  This project assumes that default user `debian` is used. Any password can be used.

### Connect to WiFi
The `connman` utility is used to handle WiFi connections on the beaglebone. The commands to connect
to a local WiFi network are as follows:

```bash
debian@beaglebone:~$ connmanctl
connmanctl> enable wifi
connmanctl> scan wifi
Scan completed for wifi
connmanctl> services
*AO RoyOwens             wifi_9884e3e23fae_526f794f77656e73_managed_psk
*AR Wired                ethernet_be6d4d656e3d_cable
    ATT8QiH6nS           wifi_9884e3e23fae_41545438516948366e53_managed_psk
                         wifi_9884e3e23fae_hidden_managed_psk
    atthome              wifi_9884e3e23fae_617474686f6d65_managed_psk
    google               wifi_9884e3e23fae_676f6f676c65_managed_psk
    Starbucks WiFi       wifi_9884e3e23fae_537461726275636b732057694669_managed_psk
    ATTUxQ5Xi2           wifi_9884e3e23fae_41545455785135586932_managed_psk
    wifi-2.4G-2B8C       wifi_9884e3e23fae_776966692d322e34472d32423843_managed_none
    SpectrumSetup-CA     wifi_9884e3e23fae_537065637472756d5
connmanctl> connect wifi_9884e3e23fae_526f794f77656e73_managed_psk
connmanctl> exit
```

In this example, the beaglebone is connecting to the `RoyOwens` network. The `connect` command uses the key adjacent to
network we want to connect to

> **_NOTE:_**  When typing the network key, tab out for auto-completion!

### Update aptitude packages.
This will take some time if it's the first upgrade since flashing.

```bash
debian@beaglebone:~$ sudo apt update
debian@beaglebone:~$ sudo apt upgrade
```

### Build and Deploy
Build and deploy flyMS to beagleboard. [See Build section](#build-environment)

### Install flyMS services:
After flyMS is built and deployed to the beagleboard, run the following script:

```
debian@beaglebone:~$ sudo ~/bin/install_flyMS.sh
```

A reboot is required after installing. When it's completed, all the flyMS services will be active

### Connect to the flyMS Webserver
TODO: explain this section!


### Calibrate Sensors and Hardware
From the flyMS webserver, calibrate all sensors and hardware
    * gyroscopes
    * accelerometers
    * magnetometer
    * DSM (remote control commands)
    * ESCs (if the hardware requires this)
