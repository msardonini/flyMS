# Hardware

Objects in the hardware module 'own' a specific piece of hardware on the Beaglebone, and use the RAII principle to
handle initialization and cleanup tasks. They provide a simpler interface of how to get/send information using
the hardware

## Inertial Measurement Unit - IMU

```{eval-rst}
.. doxygenclass:: flyMS::Imu
   :members:
```
## Programmable Realtime Unit - PRU

```{eval-rst}
.. doxygenclass:: flyMS::PruManager
   :members:
.. doxygenclass:: flyMS::PruRequester
   :members:
```
## Setpoint Module

```{eval-rst}
.. doxygenclass:: flyMS::Setpoint
   :members:
```
