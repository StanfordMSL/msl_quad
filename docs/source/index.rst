===========================
MSL Quadrotor Documentation
===========================

.. meta::
    :description lang=en: Documentation for code related to the MSL Quadrotors used at Stanford University.

.. This toctree part is needed if you want to have your content visible in the
   sidebars. Here it is hidden since the main page doesn't need to show the TOC
   when it will always be available on the left-hand side of the browser.

.. toctree::
   :maxdepth: 2
   :titlesonly:
   :hidden:

   HardwareDocs/index_hardware
   SoftwareDocs/index_software
   external_resources
   references

.. Actual summary of our documentation/project

.. TODO: Clean up this section?

Quadrotor aerial robot developed at Multi-Robot Systems Lab.

This repository contains `CAD files`_, and code for low-level quadrotor controls
and trajectory following through the mavros_ interface.

For high-level trajectory planning and generation, please refer to our
QuadsManip_ repository.

.. _cad files: https://github.com/StanfordMSL/msl_quad/tree/master/Hardware
.. _mavros: http://wiki.ros.org/mavros
.. _QuadsManip: https://github.com/StanfordMSL/QuadsManip

Features
========

- .. TODO: List our current features here

Installation
============

.. TODO: Definitely need to verify these are accurate before we finalize the
   merge.

Software Versions
-----------------

- px4 firmware: v1.7.3
- mavros: 0.23 or later
- mavlink: 2018.2.2 or later

Dependencies
------------

- glog: https://github.com/ethz-asl/glog_catkin
- ros_vrpn_client: https://github.com/StanfordMSL/ros_vrpn_client
- Eigen3

Compiling
---------

    .. TODO: Add installation instructions if easy, otherwise let's make a link

Usage
=====

Common
------

- Start mavros and VRPN for mocap: ```roslaunch mslquad quad_vrpn.launch```

- May need to change ```vrpn_server_ip``` in ```launch/vrpn.launch``` to the ip address of the mocap machine.

Trajectory Following
~~~~~~~~~~~~~~~~~~~~

- A basic trajectory following controller is implemented in **src/px4_base_controller.cpp**.

    ```roslaunch mslquad default_controller.launch```

- The controller accepts trajectory from topic ```command/trajectory```, which is of type *MultiDOFJointTrajectory*. It's important that the trajectory is updated at least 10Hz since the controller uses a simple lookahead strategy.

- At the moment, the controller operates at a fixed height (only works for 2D trajectory). 3D trajectory is TODO. 

Tests
~~~~~

- Test position control: 

    ```roslaunch mslquad pos_ctrl_test.launch```

- Test velocity control: 
    
    ```roslaunch mslquad vel_ctrl_test.launch```

- Experimental: test direct motor control (se3 control). This requires <a href="https://github.com/StanfordMSL/Firmware/tree/msl-quads-manip" target="_blank">custom PX4 firmware</a>:

    ```roslaunch mslquad se3controller.launch```

Demo Videos
===========

.. TODO: Keep this single video link, add more, or replace with a page of
   demo videos (like a blog page?)

.. image:: https://img.youtube.com/vi/yH0KMWm9cNU/0.jpg
    :target: https://youtu.be/yH0KMWm9cNU/

Related Papers
==============

If you find our work useful in your research, please consider citing:

Hardware & Experiments
----------------------

- Z. Wang, R. Spica and M. Schwager, “Game Theoretic Motion Planning for
  Multi-Robot Racing,” In Proc. of the International Symposium on Distributed
  Autonomous Robotics Systems (DARS 18), October, 2018.
  `PDF <https://msl.stanford.edu/sites/default/files/wang-etal-dars18-mlt-rbt-racing.pdf>`_

Trajectory Generation
---------------------

- Z. Wang, S. Singh, M. Pavone and M. Schwager, “Cooperative Object Transport in
  3D with Multiple Quadrotors using No Peer Communication,” In Proc. of the
  International Conference on Robotics and Automation (ICRA), pp. 1064-1071,
  2018.
  `PDF <https://msl.stanford.edu/sites/default/files/wang.singh_.pavone.ea_.icra18.pdf>`_

Differential Flatness
---------------------

- D. Zhou, Z. Wang and M. Schwager, “Agile Coordination and Assistive Collision
  Avoidance for Quadrotor Swarms Using Virtual Structures,” IEEE Transactions on
  Robotics, vol. 34, no. 4, pp. 916-923, 2018.
  `PDF <https://msl.stanford.edu/sites/default/files/zhou-etal-tro18-structure.pdf>`_

- D. Zhou and M. Schwager, “Vector Field Following for Quadrotors using
  Differential Flatness,” In Proc. of the International Conference on Robotics
  and Automation (ICRA), pp. 6567-6572. 2014.
  `PDF <https://msl.stanford.edu/sites/default/files/zhouschwagericra14quadvectorfield.pdf>`_

Contributing
============

- Report bug or request feature by opening issues on Github

- Contribution is very welcome. Please fork the project and submit pull
  requests. New code will be reviewed before merging into the codebase.

- Common functionality should be implemented in
  ```src/px4_base_controller.cpp```. New controller should derive from the base
  controller, and override the ```controlLoop()``` function.

- If contributing to documentation, this cheatsheet_ can help if you are
  unfamiliar with Markdown or RST.
  
.. _cheatsheet: https://hyperpolyglot.org/lightweight-markup

Support
=======

.. TODO: How are we handling this? Anything yet?

License
=======

The project is licensed under the MIT license.