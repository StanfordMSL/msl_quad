===========================
MSL Quadrotor Documentation
===========================

.. meta::
  :description lang=en: Documentation for code related to the MSL Quadrotors
    used at Stanford University.

.. This toctree part is needed if you want to have your content visible in the
   sidebars. Here it is hidden since the main page doesn't need to show the TOC
   when it will always be available on the left-hand side of the browser.

.. toctree::
   :maxdepth: 2
   :titlesonly:
   :hidden:

   /quickstart/index
   /hardware/index
   /software/index
   /contributing/index
   /resources
   /references

Quadrotor aerial robot developed at Multi-Robot Systems Lab.

.. image:: https://raw.githubusercontent.com/StanfordMSL/msl_quad/master/Hardware/quadrotor_rendering.jpg
  :width: 400
  :height: 300

This repository contains `CAD files`_, and code for low-level quadrotor controls
and trajectory following through the mavros_ interface.

For high-level trajectory planning and generation, please refer to our
QuadsManip_ repository.

.. _cad files: https://github.com/StanfordMSL/msl_quad/tree/master/Hardware
.. _mavros: http://wiki.ros.org/mavros
.. _QuadsManip: https://github.com/StanfordMSL/QuadsManip

Installation
============

Please review our :doc:`/quickstart/index` to get started. 

Features
========

- Provides an extensible framework for interfacing with PX4 based flight
  controllers.

- Provides a ROS waypoint following interface.

- Integrated in-flight safety checks with autoland on failures. 


Software Versions
=================

- px4 firmware: v1.7.3
- mavros: 0.23 or later
- mavlink: 2018.2.2 or later

Dependencies
============

- `glog <https://github.com/ethz-asl/glog_catkin>`_
- `ros_vrpn_client <https://github.com/StanfordMSL/ros_vrpn_client>`_
- `Eigen3 <http://eigen.tuxfamily.org/>`_

Demo Videos
===========

.. image:: https://img.youtube.com/vi/yH0KMWm9cNU/0.jpg
    :target: https://youtu.be/yH0KMWm9cNU/

.. image:: https://img.youtube.com/vi/INr92G_qOls/0.jpg
    :target: https://youtu.be/INr92G_qOls

Related Papers
==============

Lab research using the MSL Quadrotors can be found in the
`References section <references.html>`_.

Contributing
============

- Contribution is very welcome. Please fork the project and submit pull
  requests. New code will be reviewed before merging into the codebase.

- If contributing to documentation, this cheatsheet_ can help if you are
  unfamiliar with Markdown or RST.
  
.. _our github: https://github.com/StanfordMSL/msl_quad/issues
.. _cheatsheet: https://hyperpolyglot.org/lightweight-markup

Support
=======

Request help, report bugs, or request features by opening issues on
`our Github`_

License
=======

The project is licensed under the MIT license.