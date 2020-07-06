=================
Quick Start Guide
=================

Last modified: June 2020

In general, there are three software platforms involved in an mslquad setup; the flight controller,
the companion computer and the ground control station.

Ground Station Setup
--------------------
<include motive. use motive wiki/stanford flightroom docs> 
This assumes you going to run Ubuntu on the ground station. Well, you should. <work in progress>

Companion Computer
~~~~~~~~~~~~~~~~~~
Next, we need to configure the serial port to be able to take in data from the companion computer. <Need to check
the parameters for this. I think it's changed from what was shown in the github. PX4 now does it semi-automated>.

Flight Controller Setup
-----------------------
The flight controller is responsible for low-level control and is usually separate from the companion
computer to ensure a robust, real-time control output. There are two popular platforms for this; PX4
and Ardupilot. In the case of the mslquad, it is recommended that you use the former. Software installation 
is done via the QGroundControl app. Please refer to the relevant documentation sites:

1. https://docs.qgroundcontrol.com/en/
2. https://docs.px4.io/v1.9.0/en/

for initial setup. Within and after the installation of the PX4 firmware, we will need to make several
parameter value changes that reflect the choice of quadcopter hardware, companion computer and motion
capture environment.

Current PX4 version in use: v1.9

Quadcopter Hardware
~~~~~~~~~~~~~~~~~~~
For the airframe choice when installing the PX4 firmware, choose the Quadrotor x configuration with DJI
Flame Wheel F450 parameters.

Motion Capture Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~
If your test setup involves motion capture, you will need to enable onboard sensor fusion with said motion capture
environment. Once firmware installation is complete, access the flight controller's parameters through QGroundControl
and change the following:

* SYS_MC_EST_GROUP = ekf2
* EKF2_AID_MASK = 24
* EKF2_BARO_GATE = 0
* EKF2_EVP_NOISE = 0.01
* EKF2_EV_GATE = 500
* EKF2_HGT_MODE = vision

