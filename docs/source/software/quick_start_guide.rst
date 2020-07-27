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
simplicity, we have prepared disk images that can be cloned via Clonezilla
straight onto new boards. They currently come in three different flavours:

1. Odroid
2. Upboard
3. TX2

.. TODO:
    Link to cloned image for those not in lab.

Borrow the relevant USB stick from the lab.

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