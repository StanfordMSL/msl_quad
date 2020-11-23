===================
Flight Module Unit
===================

This runs direct flight control. Given the safety and real-time requirements
for control, this is a discrete unit from the companion computer. This is
usually a unit from the Pixhawk family of flight controllers running PX4.

**Current Firmware Version: PX4 v1.9**

For the flight controller, software installation and configuration is done via
the QGroundControl app. Please refer to the relevant documentation sites:

1. https://docs.qgroundcontrol.com/en/
2. https://docs.px4.io/v1.9.0/en/

for initial setup. Within and after the installation of the PX4 firmware, we
will need to make several parameter value changes with QGroundControl that
reflect the choice of quadcopter hardware, companion computer and motion
capture environment.

Quadcopter Hardware
-------------------

For the airframe choice when installing the PX4 firmware, choose the Quadrotor
x configuration with DJI Flame Wheel F330 parameters. If you are using a
different frame, choose the closest approximation. Amongst other parameters,
this affects the PID values on the craft.

.. image:: /_static/images/software/airframe.png
  :target: ../_static/images/software/airframe.png
  :width: 310px
  :alt: Airframe selection 
  :align: center

Flight Modes
--------------

The correct flight modes must be configured in QGroundControl to use the
handheld controller properly. 

.. image:: /_static/images/software/flightmodes.png
  :target: ../_static/images/software/flightmodes.png
  :width: 930px
  :alt: Airframe selection 
  :align: center

Safety
-------

Configure the following safety paramters.

.. image:: /_static/images/software/safety.png
  :target: ../_static/images/software/safety.png
  :width: 930px
  :alt: Airframe selection 
  :align: center

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

.. image:: /_static/images/software/ekf1.png
  :target: ../_static/images/software/ekf1.png
  :width: 930px
  :alt: ekf settings 
  :align: left

.. image:: /_static/images/software/ekf2.png
  :target: ../_static/images/software/ekf2.png
  :width: 930px
  :alt: ekf settings 
  :align: right