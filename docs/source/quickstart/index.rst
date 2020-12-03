=================
Quick Start Guide
=================

.. meta::
    :description lang=en: This section provides a step-by-step guide working
        with the `mslquad` hardware and software

.. toctree::
   :maxdepth: 2
   :titlesonly:
   :hidden:
   
   ground_station
   companion_computer
   FMU_computer

Setup
======

There are three distinct computer groups that require attention during setup:

1. :doc:`ground_station` **(GS)**:
   All involved computer units that are on the ground. In most cases, this
   would be ``relay.local``, ``mocap.local``, and your own personal computer.
2. :doc:`companion_computer` **(CC)**:
   Computer on-board the quad used for direct access to prototype hardware and algorithms. This
   is usually a Linux machine with direct access to at least one of the Ground
   Station units.
3. :doc:`FMU_computer` **(FC)**: 
   This runs direct flight control. Given the safety and real-time requirements
   for control, this is a discrete unit from the companion computer. This is
   usually a unit from the Pixhawk family of flight controllers running PX4.


Flight
=======

.. important::
    Quadrotors are dangerous and serious injury may occur from incorrect
    operation. Proceed with extreme caution. *It's better to be on the ground
    wishing you were the air, than in the air wishing you were on the ground.*

1. Turn on Motive_ on the ``mocap`` machine. 
2. Set rigid body of the quad with the correct orientation. Verify that the x-axis of the quad is aligned with the long axis of the room and all IR reflection spheres are being registered in motive.
3. Turn on hand-held control (see :doc:`/hardware/remote_controllers`).
4. Check Lipo battery with battery checker.
5. Plug into quad (PX4 initialization music will play).
6. ``ssh`` into quad.
7. Start the software by running ``roslaunch mslquad quad_vrpn.launch`` and
   your controller (see :doc:`/software/index`).
8. Review safety checklist below.

.. _Motive: https://optitrack.com/software/motive/

Safety Checklist
==================

.. important::
    Make sure you complete the following check list *before every flight*. Most
    of the checks are automated by the controller on startup, but that is no substitute for good ol human caution 

Hardware
----------

1. Mechanical

    a. All *4* rotors are tightly secured (more than hand tight)
    b. Battery is secured

2. Electrical

    a. The **FCU** serial connection is secured between the px4 and **CC**
    b. Battery voltage is over the nominal 11.5v (*DO NOT* start a flight with
       less than this voltage)
    c. Keep a battery checker handy

Software
--------

1. Motive

    a. z is up
    b. quad frame is aligned with room (room's long-axis is quad x-axis)

2. ROS Topics

    a. ``local_position/pose`` matches ``vision_pose/pose``
    b.  ``state`` has ``connected=True``
    
3. PX4 Arms and Kills

    a. Arm the quad by pinching the sticks down and inwards. DO NOT push up
       throttle stick after arm as this will cause the craft to take off.
    b. Gently push the throttle stick up until the motors start spinning. check
       that the propeller rotation is in the correct direction.
    c. Disarm the quad by pinching the sticks down and inwards.
    d. Arm the quad again but this time, use the kill switch to disarm. The
       motors should cut instantaneously.
