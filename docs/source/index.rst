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

   /quickstart/index
   /hardware/index
   /software/index
   /contributing/index
   /external_resources
   /references

.. Actual summary of our documentation/project

.. TODO: Clean up this section?

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


Getting Started
---------------

Please review our :doc:`quickstart/index` to get started. 

.. Features
.. ========

.. - .. TODO: List our current features here

.. Installation
.. ============

.. .. TODO: Definitely need to verify these are accurate before we finalize the
..    merge.

Software Versions
-----------------

- px4 firmware: v1.7.3
- mavros: 0.23 or later
- mavlink: 2018.2.2 or later

Dependencies
------------

- `glog <https://github.com/ethz-asl/glog_catkin>`_
- `ros_vrpn_client <https://github.com/StanfordMSL/ros_vrpn_client>`_
- `Eigen3 <http://eigen.tuxfamily.org/>`_

.. is this all unneeded or wrong
.. Compiling
.. ---------

..     .. TODO: Add installation instructions if easy, otherwise let's make a link

.. Usage
.. =====

.. Common
.. ------

.. - Start mavros and VRPN for mocap: ```roslaunch mslquad quad_vrpn.launch```

Demo Videos
===========

.. TODO: Keep this single video link, add more, or replace with a page of
   demo videos (like a blog page?)

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

- Report bugs or request features by opening issues on `our Github`_

- Contribution is very welcome. Please fork the project and submit pull
  requests. New code will be reviewed before merging into the codebase.

.. this is too detailed for this level of page
.. - Common functionality should be implemented in
..   ```src/px4_base_controller.cpp```. New controllers should derive from the base
..   controller, and override the ```controlLoop()``` function.

- If contributing to documentation, this cheatsheet_ can help if you are
  unfamiliar with Markdown or RST.
  
.. _our github: https://github.com/StanfordMSL/msl_quad/issues
.. _cheatsheet: https://hyperpolyglot.org/lightweight-markup

Support
=======

.. TODO: How are we handling this? Anything yet?

License
=======

The project is licensed under the MIT license.