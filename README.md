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

## Communication with the Beaglebone

For flashing software or accessing the [flyMS Webserver](#flyMS-Webserver), the network (usually WiFi) is used. For
in-flight navigation commands, a Spektrum RC and
[digital receiver](https://www.spektrumrc.com/Products/Default.aspx?ProdID=SPM9645) are used.

## flyMS Webserver

The flyMS Webserver is run on the beaglebone itself, and is used to dynamically make configuration changes or update
calibration.

For example, if you want to iterate on PID constants in between flights, you can use the flyMS Webserver. On a smart
phone, connect to the wifi network broadcasted by the beaglebone, navigate to to the URL of the webserver, and make
modifications to PID constants between flights. This helps accelerate the process of getting a smooth flight.
