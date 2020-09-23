=================
Flight Module Unit
=================

This runs direct flight control. Given the safety and real-time requirements
 for control, this is a discrete unit from the companion computer. This is
 usually a unit from the Pixhawk family of flight controllers running PX4.

**Current Firmware Version: PX4 v1.9**

For the flight controller, software installation and configuration is done via
the QGroundControl app. Please refer to the relevant documentation sites:

1. https://docs.qgroundcontrol.com/en/
2. https://docs.px4.io/v1.9.0/en/

for initial setup. Within and after the installation of the PX4 firmware, we
will need to make several parameter value changes that reflect the choice of
quadcopter hardware, companion computer and motion capture environment.

Quadcopter Hardware
-------------------

For the airframe choice when installing the PX4 firmware, choose the Quadrotor
x configuration with DJI Flame Wheel F330 parameters. If you are using a
different frame, choose the closest approximation. Amongst other parameters,
this affects the PID values on the craft.

Motion Capture Environment
--------------------------

If your test setup involves motion capture, you will need to enable onboard
sensor fusion with the flight room's motion capture environment. Once firmware
installation is complete, access the flight controller's parameters through
QGroundControl and change the following:

* SYS_MC_EST_GROUP = ekf2
* EKF2_AID_MASK = 24
* EKF2_BARO_GATE = 0
* EKF2_EVP_NOISE = 0.01
* EKF2_EV_GATE = 500
* EKF2_HGT_MODE = vision