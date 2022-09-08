# Introduction

flyMS is a flight stack used to control a quadrotor drone. It is run on a
[Beaglebone Blue](https://beagleboard.org/blue), or Beaglebone Black equipped with the
[RoboticsCape](https://www.newark.com/element14/bb-cape-robotics/robotics-cape-9-18-vdc/dp/95Y0637).
The stack is written in C++, and is easy to read and modify.

## Communication with the Beaglebone

For flashing software or accessing the [flyMS Webserver](#flyMS-Webserver), the network (usually WiFi) is used. For
in-flight navigation commands, a Spektrum RC and
[digital receiver](https://www.spektrumrc.com/Products/Default.aspx?ProdID=SPM9645) are used.

## flyMS Webserver

The flyMS Webserver is run on the beaglebone itself, and is used to dynamically make configuration changes or updates
calibration.

For example, if you want to iterate on PID constants in between flights, you can use the flyMS Webserver. On a smart
phone, connect to the wifi network broadcasted by the beaglebone, navigate to to the URL of the webserver, and make
modifications to PID constants between flights. This helps accelerate the process of getting a smooth flight.