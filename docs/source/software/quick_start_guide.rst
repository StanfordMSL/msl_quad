=================
Quick Start Guide
=================

There are three distinct computer groups that require attention during setup:

1. **Ground Station (GS)**:
   All involved computer units that are on the ground. In most cases, this
   would be relay.local, mocap.local and your own personal computer.
2. **Companion Computer (CC)**:
   Computer used for direct access to prototype hardware and algorithms. This
   is usually a Linux distro with direct access to at least one of the Ground
   Station units.
3. **Flight Controller (FC)**: 
   This runs direct flight control. Given the safety and real-time requirements
   for control, this is a discrete unit from the companion computer. This is
   usually a unit from the Pixhawk family of flight controllers running PX4.

.. TODO:
    Generate quickstart script and usb sticks

Ground Station Setup
====================

Setup for motion capture (relay.local and mocap.local) can be found
`here <https://stanfordflightroom.github.io/documentation>`_. In most cases,
users need only make changes on mocap.local. On your own personal computer,
ensure you have the following software:

1. `ROS (melodic) <http://wiki.ros.org/melodic/Installation/Ubuntu>`_:
   For interface with mocap.local and CC during flight.
2. `QGroundControl <http://qgroundcontrol.com/downloads/>`_:
   For setup of flight controller.
3. `Terminator <https://gnometerminator.blogspot.com/p/introduction.html>`_:
   It makes the multitude of terminals that you will be needing a little
   tidier.

Within ROS, you might need certain packages and associated software (e.g the
vrpn client, rviz, gazebo) and so a full ROS-desktop install is advised. For
reference, you could use the following script:

.. code-block:: shell

    sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-vrpn-client-ros
    sudo wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    ./install_geographiclib_datasets.sh


Companion Computer Setup
========================

You will need to run a similar setup on the companion computer (CC). For
simplicity, we have prepared disk images that can be cloned via Etcher
straight onto new boards. We currently use the ODriod UX4 as our companion
computer.

Instructions for flashing can be found
:ref:`here<software/Companion_computer:Companion Computer>`.

Once you have flashed the relevant image and AFTER you have setup the flight
controller (see next section) you will need to configure one of the board's
serial ports for communication with the flight controller.

.. TODO:
    Generate quickstart script and usb sticks

Flight Controller Setup 
=======================

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
x configuration with DJI Flame Wheel F450 parameters. If you are using a
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


Safety Checklist
==================

.. important::

    Make sure you complete the following check list *before every flight*. Most
    of the checks are automated by the controller on startup, but that is no
    substitute for good ole human caution.


Hardware
----------

1. Mechanical

    a. All *4* rotors are tightly secured (more than hand tight)
    b. Battery is secured

2. Electrical

    a. The fcu serial connection is secured between the px4 and cc
    b. Battery voltage is over the nominal 11.5v (*DO NOT* start a flight with
       less than this voltage)


Software
--------

1. Motive

    a. z is up
    b. quad frame is aligned with room (room's long-axis is quad x-axis)

2. ROS Topics

    a. ``local_position/pose`` matches ``vision_pose/pose``
    b.  ``state`` has ``connected=True``
    
3. PX4 Arms and Kills
    a. Arm the quad by pinching the sticks down and inwards. DO NOT push up throttle stick after arm as this will cause the craft to take off.
    b. Gently push the throttle stick up until the motors start spinning. check that the propeller rotation is in the correct direction.
    c. Disarm the quad by pinching the sticks down and inwards.
    d. Arm the quad again but this time, use the kill switch to disarm. The motors should cut instantaneously.
