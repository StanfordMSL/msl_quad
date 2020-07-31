==================
Remote Controllers
==================

.. meta::
    :description lang=en: Hardware page for the quadrotor's remote control transmitter.

The quadrotors require a standard remote control transmitter to do the following. 

#. Arm or disarm the flight controller
#. Toggle between flight control modes
#. Fly manually using the transmitter
#. Perform emergency landings

We use the Spektrum DX6e 6-channel transmitter. This page documents the :ref:`configuation <remote-controllers-configuration>` of this transmitter and its :ref:`use <remote-controllers-use>` with the quadrotors. 

.. _remote-controllers-configuration:

Remote Controller Configuration
===============================

There are three transmitter features to be familiar with: 

#. Control sticks
#. Flight mode toggle
#. Throttle cut toggle

.. important::
  The default position for all toggles is the **0** position (pointed furthest away from the user). This is depicted in the image below. Ensure all toggles are set to the **0** position before using the controller. 

Control Sticks
--------------

The control stick control the throttle, roll, pitch, and yaw of the quadrotor. The *zero configuration* is illustrated in the image below where all sticks are at the center of their range. 

Flight Mode Toggle
------------------

The flight mode toggle switches between stability mode (**0**), position hold mode (**1**), and offboard mode (**2**). More discussion of these modes is in the :ref:`remote controller use <remote-controllers-use>` section below.  

Throttle Cut Toggle
-------------------

The throttle cut toggle does nothing in the **0** position and cuts the throttle in the **1** position. This is an emergency method of stopping the rotors from spinning. 

.. warning::
  **Do not** cut throttle in flight as the quadrotor will drop out of the air. 

.. image:: /_static/images/hardware/remote_controllers_front.jpg
  :target: ../_static/images/hardware/remote_controllers_front.jpg
  :width: 310px
  :alt: Spektrum Configuration Front
  :align: left

.. image:: /_static/images/hardware/remote_controllers_top.jpg
  :target: ../_static/images/hardware/remote_controllers_top.jpg
  :width: 310px
  :alt: Spektrum Configuration Top
  :align: right


.. _remote-controllers-use:

Remote Controller Use
=====================

.. important::
   Please read this entire section before attempting to fly a quadrotor -- even autonomously -- as errors can cause serious injury. 

Basic Flight Procedures
-----------------------

We summarize the basic use of the transmitter for quadrotor flight and refer the reader to the :doc:`quick start guide <../software/quick_start_guide>` for additional information on setting up the quadrotor. 

Preflight Check
^^^^^^^^^^^^^^^

#. Ensure throttle is all the way down (not in the zero configuration)
#. Ensure all switches are set to the default **0** position
#. Launch quadrotor mocap ROS nodes (the quadrotor cannot fly without pose feedback from mocap)

Arm Procedure
^^^^^^^^^^^^^

#. Perform preflight check
#. Arm quadrotor's safety button located on the flight controller
#. With thottle down, slide yaw control all the way to the left and hold for 2 seconds
#. The drone is now armed and ready for flight

Disarm Procedure
^^^^^^^^^^^^^^^^

#. With thottle down, slide yaw control all the way to the right and hold for 2 seconds
#. Disarm quadrotor's safety button located on the flight controller
#. The drone is now safe to be handled

Emergency Landing Procedure
^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Switch the quadrotor to stability flight
#. Gently decrease the throttle to land
#. [Optional] Cut throttle by setting the throttle cut toggle to the **1** position
#. Disarm the quadrotor

Flight Mode Procedures
----------------------

We summarize the instructions for using each flight mode with the quadrotor. Additional information on each flight mode can be found in the `PX4 flight mode documentation`_.

.. _PX4 flight mode documentation: https://docs.px4.io/v1.9.0/en/flight_modes/#multicopter

Stability Flight
^^^^^^^^^^^^^^^^
Stability mode enables you to fly the quadrotor manually using the transmitter as you would expect on a normal drone. Here, the throttle stick position is proportional to thrust. To fly manually in stability mode:

#. Arm the quadrotor
#. Take off by gently increasing the throttle

Position Hold Flight
^^^^^^^^^^^^^^^^^^^^

Position hold flight uses mocap pose feedback to keep the quadrotor hovering at desired waypoints. These waypoints are set using the transmitter's control sticks. 

.. warning::
  All sticks **must** be set in the zero configuration (in the middle of their range). Deviation from the zero configuration modifies the new waypoint. For example, if the quadrotor is hovering and you increase the thottle stick from the zero configuration upwards, the quadrotor will fly higher. This is *different* from stability flight. 

To fly in position hold mode:

#. Arm the quadrotor
#. Flip flight mode toggle to the **1** position
#. Ensure all sticks are in the zero configuration
#. Take off by gently increasing the throttle
#. Remember to return the sticks to the zero configuration if you want the quadrotor to hover
#. Land by decreasing the throttle gently

Offboard Flight
^^^^^^^^^^^^^^^

Offboard flight allows the quadrotor to maneuver according to external commands. 

.. warning::
  The sticks on the transmitter will **not** respond in this mode. It is important to be prepared to switch out of offboard mode when testing autonomous flight in case there is an emergency and you need to land manually (either in position hold or stability mode). 

To fly in offboard mode:

#. Arm the quadrotor
#. Flip flight mode toggle to the **2** position (the quadrotor is now awaiting commands)
#. Launch external command node (be careful as the quadrotor will immediately respond)

