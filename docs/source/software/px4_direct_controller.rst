=====================
PX4 Direct Controller
=====================

Here be progress logs for the development of a state/input trajectory planning pipelinefor quadcopters using ROS and PX4. Perhaps this will get far enough that I'll polish it into a full fledged tutorial/manual.

Objective
---------
The objective is to implement a trajectory planner that can fly a quadcopter through a tightly constrained gate. 

Hardware
--------
Our hardware setup is as follows, a racing quadcopter equiped with two onboard computers: 1) a PixRacer running a modification of the PX4 firmware. 2) an UpBoard (an Intel® ATOM™ x5-Z8350) running ROS Melodic (on Ubuntu 18.04) with modified mavros and mavlink packages. We use this setup to get better parity with an existent simulation setup (https://github.com/StanfordMSL/quadrotor_sim) that is run on similiar hardware. We skip feasibility testing with other Linux flavours and CPU architectures. 

Another point on the hardware is that the focus here is the controls aspect of the pipeline so we simplify the challenge of state estimation by assuming we have access to an accurate source of the full state (i.e IMU + Mocap fusion) and we simplify communication by using a wired connection between the two computers. As will be emphasized later, we run the more time consuming part of the control pipeline separate from the rest of the control. This framework ensures that safety guarantees are focused on the low-level controller.

Pipeline
--------

.. image:: ../_static/images/px4_direct_control/pipeline.png

We split the pipeline into two key components, the "High Level Controller" and the "Low Level Controller". The job of the former is to solve and simulate the entire trajectory while the job of the latter is to execute the resultant motor commands to achieve said trajectory. We split it as such because the former can take long to solve while the latter needs to be run at a high rate for aggressive trajectories (let's say 200Hz).

As an example, let's use the differential flatness approach introduced by Mellinger. The High Level Controller would correspond to the solving of the piecewise QP problem. This produces a flat output trajectory that is fed into back into the differential flatness transform (the Low Level Controller) to generate the wrench input necessary. Note that this architecture is general and the same partition can be applied to many other approaches (e.g for iLQR High Level would generate the feedback and feedforward matrices and the Low Level would generate the input wrench using said matrices and an estimate of current state). For clarity, I shall define the wrench to be thrust (Fz) and roll, pitch and yaw torques (Mx, My, Mz respectively). Let us also define the intermediate variables (e.g the flat outputs or the feedback and feedforward matrices) as the trajectory nominal (traj henceforth). This name is based on the fact that these values are the product of a nominal trajectory that was simulated using the High Level Controller.

In PX4 Architecture
-------------------
We want to implement this into PX4 so that we can leverage its already existing safety and state estimation features. Given the high rates need for control and the necessary communication between a variety of devices (computers and sensors both onboard and offboard), developing a test platform from the ground up would be too time consuming. 

So here is where it fits into the PX4 architecture.

.. image:: ../_static/images/px4_direct_control/px4_modified.png

Seems deceptively simple. But underneath those blocks, there's a bunch of work across three different repos that we need to implement 1) ROS, 2) mavros/mavlink and 3) PX4 itself. Thankfully, with the right approach, we need only implement the pipeline once for the mavros/mavlink and PX4 components, leaving the customization (specifically... the implementation of our research work) of the high-level controller as purely a ROS problem.

To add some detail to what is meant by 'the right approach', we want something that can exist in the PX4/mavros/mavlink repositories. This way we can keep this pipeline usable across updates to PX4 itself (given its much broader objectives).

Development Roadmap
-------------------
We will take the approach of developing from the PixRacer up to the Upboard; from low-level controller up to the high-level controller. As such, the roadmap (and updates in a somewhat chronological order) are as follows:

[DONE] Direct Access to the Wrench Input
++++++++++++++++++++++++++++++++++++++++
Fortunately for us, there already exists a method for accessing the wrench input, the uORB topic actuator_controls_0. It's in non-dimensional form so some kind of scaling is necessary but we'll worry about that in a later hardware test. To implement this we implement a submodule (mc_direct_control). The objective of this module is to subscribe to an arbitrary uORB topic (say... rc_channels) and spit out a wrench input that is a function of said uORB topic.

Some additional work is done at compilation to keep this separate from the default pipeline. Ideally we want a specific build and boot script but for now what I do is piggyback the multicopter build script and then change the boot (via the SD card /etc/ method) to kill the original mc_rate_control module and replace it with the mc_direct_control module.

[DONE] Introduce a custom uORB topic for traj
+++++++++++++++++++++++++++++++++++++++++++++
The problem here is that traj can take a variety of forms and to maintain generality while still being efficient with communication bandwidth, I'm not sure what is the 'right approach' here. For now, I assume we are going with the diff. flat implementation, which requires traj to be a message with a time stamp and 20 float32 numbers representing the flat outputs (x,y,z and psi) and their 0th to 4th order derivatives. All other parameters necessary to calculate the input wrench will be hardcoded into mc_direct_control (to be converted to a loading at initialization later) to save bandwidth.

[WIP] Test Bandwidth Capacity of traj data
++++++++++++++++++++++++++++++++++++++++++
Ideally, we want the wrench input to be updated at 200Hz. But I'm not sure whether the hardware can send the traj message at 200Hz. I also figure there will be some timing features needed to update the variables as we run the low-level controller. This is where I'm at now. Should the bandwidth prove to be insufficient, there are some workarounds we can consider: 1) compressing the flat output data into a smaller message 2) storing the entire trajectory in memory before executing it 3) use the FastRTPS bridge (PepMS's method). We'll see how this goes.

[TODO] Parameter Estimation for Current Platform
++++++++++++++++++++++++++++++++++++++++++++++++
To do a hardware test, we'll need to get reliable estimates of things like motor thrust coefficients and the mass properties.

[TODO] Implement a Simple Hover with Failsafe Verification
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
I imagine that rough values of the above parameters should get us to do a rough hover. This shall be used as a test for the failsafe. I have decided to use a complete motor cut (instead of an attempt to restore the craft to a stationary hover) for simplicity. Besides... If we're going to get to the linear speeds that I personally want to hit... a hover recovery would be no better.

[TODO] Tidying Up
+++++++++++++++++
Writing the actual custom build and boot script. Assigning the parameters at initialization instead of hardcoding it. Cleaning up implementation for an actual PR into the relevant repositories.

[TODO] Aggressive Trajectories
++++++++++++++++++++++++++++++
Try flying through gates.

Implementation Thoughts
=======================
1) The idea of splitting the controller into high-level + low-level can be generalized a little further. One need not follow the above division of (simulation of full trajectory) + (unpack data and converty using fast updates of state estimation). The generalization really is (slow control [<200Hz]) + (fast control [200Hz]). How that is done is up to the designer.
2) The guide for implementing custom uORB messages (https://dev.px4.io/v1.11/en/middleware/mavlink.html) is out of date. I think it's also more focused on getting messages from the PixRacer to the companion computer. I used the tutorial from (https://dev.px4.io/master/en/ros/mavros_custom_messages.html) which is still out of date but more detailed and closer to the latest version of the Firmware. Also of use was referencing other uORB messages in (https://github.com/mavlink/mavros/tree/master/mavros/src/plugins).

Development Repositories
========================
PX4 Firmware: https://github.com/lowjunen/Firmware/tree/direct_motor
mavros/mavlink/ROS: https://github.com/lowjunen/msl_dev
