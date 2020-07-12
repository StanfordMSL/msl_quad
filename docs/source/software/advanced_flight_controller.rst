==========================
Advanced Flight Controller
==========================


This page will go over how to use the trajectory tracking controller. The tracking controller is designed to follow a single trajectory from start to finish using a lower level velocity controller. This controller will run closed loop on the desired trajectory, but is **not** designed to accept a constantly changing trajectory. A trajectory can either be passed via a rostopic or as a ``.txt`` file

* Usage
* ROS Parameters

Usage
=====

* Start the ROS Bridge.
* launch ``roslaunch mslquad traj_controller.launch``

As with all launch files, make sure the target namespaces are correct. Once a trajectory is read a ``Moving to next waypoint`` message will appear. To start the trajectory a ``Time`` ROS msg needs to the sent to ``tower/scramble``.

ROS Parameters
==============

Trajectory Controller Parameters:

* ``traj_target_topic``: Rostopic MultiDOFJointTrajectoryPoint. (``command/trajectory``)
    Target trajectory of the quad
* ``load_traj_file``: Boolean. (``True``)
    Use onboard onboard text file.
* ``traj_file``: string. (``params/trajectories/traj.txt``)
    onboard trajectory file. See example file.
* ``traj_timestep``: float. (0.2)
    timestep (in sec) between sequential waypoints. 
* ``traj_Kp``: float. (``2.0``)
    Proportional gain on the velocity controller
    