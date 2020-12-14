===============
Ground Station
===============

This section is for all things involving ground-based computers that are 
supporting the quad experiments. Apart from your own personal computer, you 
also need to have Optitrack (our motion capture system running on relay.local 
and mocap.local in Stanford Flight Room) setup and streaming rigid body 
information. More information on how to setup the motion capture system can be 
found in the `Stanford Flight Room Documentation 
<https://stanfordflightroom.github.io/documentation>`_. 
In most cases, users only need to make changes on mocap.local, such as 
modifying and registering rigid bodies through the Motive Software. 

The following software needs to be installed on your own computer:

1. `ROS (melodic) <http://wiki.ros.org/melodic>`_:
   For interface with mocap.local and the Companion Computer during flight.
2. `QGroundControl <http://qgroundcontrol.com>`_:
   For setup of flight controller.
3. `Terminator <https://gnometerminator.blogspot.com/p/introduction.html>`_:
   It makes the multitude of terminals that you will be needing a little
   tidier.

ROS Melodic Installation and Setup
----------------------------------

Please see the official `installation instructions 
<http://wiki.ros.org/melodic/Installation/Ubuntu>`_. 
Note that ROS Melodic is primarily targeted at the Ubuntu 18.04 (Bionic) 
release. Setup instructions specific to Stanford Flight Room can be found 
`here <https://stanfordflightroom.github.io/start_ros>`_.

QGroundControl Installation and Setup
-------------------------------------

QGroundControl is used to configure the quads. To install and run 
QGroundControl on your personal computer, follow the Ubuntu Linux section of 
the `instructions page <https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu>`_.

Terminator Installation
-----------------------

This step is optional, although we highly recommend that you use Terminator or 
a similar tool to monitor multiple streams of ROS topics before and during 
flight. Terminator lets you arrange multiple terminals in grids and save 
layouts. See the `introduction page 
<https://gnometerminator.blogspot.com/p/introduction.html>`_ 
for more information and installation instructions.
