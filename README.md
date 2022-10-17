# flyMS - Flight Stack for Quadrotor Drone

[![Build](https://github.com/msardonini/flyMS/actions/workflows/main.yml/badge.svg)](https://github.com/msardonini/flyMS/actions/workflows/main.yml)

Check out the full [flyMS Documentation!](https://msardonini.github.io/flyMS/)

## Introduction

flyMS is a flight stack used to control a quadrotor drone. It is run on a
[Beaglebone Blue](https://beagleboard.org/blue), or Beaglebone Black equipped with the
[RoboticsCape](https://www.newark.com/element14/bb-cape-robotics/robotics-cape-9-18-vdc/dp/95Y0637).
The stack is written in C++ (except for front-end tools), and the scope is sm

This project can serve as a platform for prototyping technologies which use quadrotor drones as a vehicle. This can
include:
* Control strategies
* Digital filter design
* Autonomous navigation
    * [Visual-inertial odometry](https://en.wikipedia.org/wiki/Visual_odometry) (VIO)
    * [Simultaneous localization and mapping](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) (SLAM)
    * GPS based navigation
* Collision avoidance systems
* Route planning

By default, you can fly the drone in the standard
[Stabilized flight mode](https://docs.px4.io/main/en/getting_started/flight_modes.html#manual-stabilized-mode-mc)
with a Remote Controller commanding the roll/pitch/yaw values.
[Acro Mode](https://docs.px4.io/main/en/getting_started/flight_modes.html#acro-mode-mc)
is also supported, and with externally provided navigation data,
[Position Mode](https://docs.px4.io/main/en/getting_started/flight_modes.html#position-mode-mc) is supported.

### Related Projects

The most common open source flight stack is arguable the [PX4 Auto Pilot](https://px4.io/). It is fully featured,
supports many sensors and hardware platforms. At the time of this writing, beaglebone support is still experimental.

Another common platform is [Ardupilot](https://ardupilot.org/) which supports many vehicle types, including:
fixed wing aircraft, helicopters, drones, boats, submarines, and more.

flyMS is a platform much smaller in scope, allowing users with a Beaglebone & quadrotor hardware an alternative way to
install and develop autonomy systems at a reduced learning curve.

## Required Hardware

1. [Beaglebone Blue](https://beagleboard.org/blue), or Beaglebone Black equipped with the
[RoboticsCape](https://www.newark.com/element14/bb-cape-robotics/robotics-cape-9-18-vdc/dp/95Y0637).
2. A Linux PC
3. A Spektrum Remote Controller (DX5+) with the
[DSM Remote Receiver](https://www.spektrumrc.com/Products/Default.aspx?ProdID=SPM9645)
4. Standard quadrotor hardware:
    * Frame
    * Motors
    * Electronic Speed Controllers (ESCs). The standard 3-Pin servo connector is supported by the Beaglebone
    * Battery
5. (Optional) A smartphone, for rapid PID tuning during outdoor testing


## flyMS Webserver

flyMS includes a webserver that is used for interfacing with the flight configuration and system state. It can update
PID constants, run calibration routines, and inform you of errors. It can be accessed at http://beaglebone.local:5001/
from any device on a shared network.

> **_NOTE:_** If flight testing away from a local WiFi connection, you can use a smartphone to connect to the WiFi
network broadcasted by the beaglebone itself to access this webserver. The network name is `BeagleBone-XXXX` with a
default password of `BeagleBone`
