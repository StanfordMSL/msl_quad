=======================
Basic Flight Controller
=======================

The flight controller is responsible for passing high level position information to the pixhawk/pixracer. The controller is written in C++ and designed to be extended for multiple use cases. To develop your own controller see the controller development page.

This page will cover how to use the basic pose controller:

* `Usage`_
* `Troubleshooting`_
* `ROS Parameters`_

Usage
=====

You'll need to launch two sets of launch files **on the quad** via ``ssh``.

ROS Bridge
^^^^^^^^^^^^^
::

    roslaunch mslquad quad_vrpn.launch

This will create the connection between the roscore, motion capture, and the px4. You should verify that the quad ID in::

    quad_vrpn.launch

is the same as the one you have in Motive *e.g.,* for quad3 the ID="3".

Controller
^^^^^^^^^^^^^
::

    roslaunch mslquad pose_controller.launch

This will launch the ``pose_controller`` node which allows you to set a goal point of the quad. You can directly publish to ``quad#/command/pose``. You can also run::

    roslaunch mslquad goal_set.launch

to use a python script to set the goal position. Make sure you change the namespace before using this launch file.

Troubleshooting
===============
.. TODO: make this section a whole other page?

Things will likely break, this is here so you don't break as well. The first you should do is **restart everything on the quad and make sure everything is plugged in.**

All the examples use "quad0", change that to the quad that's not working.

Verify all of the following 

* Serial Connection
    Verify all the serial connection wires are connected and not loose. To check if px4 is comunicating with ros run::

        rostopic echo quad0/mavros/state

    This should return a ``connected: TRUE``. If not then check that ``fcu_url`` in ``quad_vrpn.launch`` is pointing to the correct port.

* Motion Capture 
    Verify that the motion capture and quad estimator poses match::
    
        rostopic echo quad0/mavros/vision_pose/pose
        rostopic echo quad0/mavros/local_position/pose

ROS Parameters
==============

Basic Parameters:
You'll find these in the ``pose_controller.launch``  file.


* ``takeoff_height``: float. (1.5)
     Takeoff height in m. Do **not** set this higher than **2**
* ``max_vel``: float. (2.0)
    Maximum velocity in m/s. The flight room is small don't go higher than 4 m/s
* ``auto_takeoff``: Boolean. (``True``)
    The quad will takeoff as soon as it's armed. **Stand back**. Don't be another statistic.

Pose Controller Parameters:

* ``pose_target_topic``: Rostopic Pose. (``command/pose``)
    Target pose of the quad.




