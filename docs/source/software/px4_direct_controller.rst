=====================
PX4 Direct Controller
=====================

Here be the ramblings of jelow as he tries to develop a full-stack pipeline for
quadcopter trajectory planning. Be warned. There is a lot of text diarrhea. If it
works out, this document will be polished into an actual proper tutorial.

Objective
---------
The idea here is to use PX4 to execute a full-stack pipeline for quadcopter trajectory
planning. The pipeline looks as follows:

Pipeline
--------

.. image:: /_static/images/px4_direct_control/pipeline.png

Trajectory objectives would be like flying through a sequence of gates. Trajectory 
constraints would be like motor limits and obstacle avoidance. These are fed into
a high level controller whose purpose is to simulate the entire trajectory and output
the corresponding motor wrench commands to achieve it. Let us define the wrench command
specific to the quadcopter to be the thrust (Fz) and the roll, pitch, yaw torques (Mx,My,Mz).
A good example of such a high level controller would be the differential flatness based
controller seen in the Minimum Snap Trajectory paper by Mellinger. Another example would be
the iterative Linear Quadratic Regulator (iLQR).

It is important here that we keep things general. Whatever the constraints, objective or
high level controller are, what we care about is only the 'traj' message that comes out of it.
traj contains the terms we need to compute the wrench command online EXCEPT FOR the quadcopter's
current state. In the case of the minimum snap controller that would be the flat output polynomial
spline. In the case of iLQR it would be the feedback and feedforward matrices. One might wonder, why
not just output the wrench command directly. Well... a problem that I am worried about is that this
high level controller is likely to have an expensive compute; i.e it'll take too long to compute to 
meet online control requirements. Let's assume the actuator controller needs to run at 200Hz. It is 
simply not robust to believe that the high level controller can solve consistently at under 0.005s.
So the workaround is simple... we update our high level solution as and when we can but at the low
level end, we use our 'current best' solution. This is essentially what traj encodes. For that 0.005s,
we lock our trajectory based on our current best prediction of the entire trajectory. This corresponds
to an already computed polynomial spline or FF+FB matrix set. These are often functions of current state,
which we can assume is reliably updated at a high rate. And so we compute the corresponding low level solution
u = f(traj,x).

In PX4 Architecture
-------------------
So... where does this fit into the px4 architecture. To keep the rest of the features running (the safety checks,
state estimation, mocap integration etc.) we do the following:

.. image:: /_static/images/px4_direct_control/px4_modified.png

Note that the high level controller can be separate. So write it in whatever flavour you fancy. What matters is that
it has to publish a uORB message that contains the traj data. This is then fed into a low level controller using the
existent Message Bus. We then replace the pos->att->rate control module sequence with our own custom module that does
the necessary computation where it subscribes to traj and x to output u. 

Implementation Thoughts
-----------------------
A few points that are worth noting.
1. Failsafe: The ONLY failsafe response that I will be encoding is a complete kill. I do not plan to do any sophisticated
recovery like a 'return to hover' etc. Reason is that I intend to fly at high speeds (where recovery is unlikely even with
perfect state estimation).
2. A submodule approach allows this type of controller to exist within the larger px4 repo. I feel that development of such
low level controls would be beneficial to the research community and so I want to implement it in a way that can be supported
in the long term. Thanks to the submodule architecture of the px4 flight software, this can be done pretty easily. We put the
low level controller as a module of its own and expose the section that does the actual f(traj,x) compute. We introduce user-definable
message types to be able to parse the traj data. And the rest (timing updates on which traj data to use etc) we implement for the
user as these are general features that would be required of any low level + high level controller.
3. Another way of seeing this high-level + low-level pairing is as division of a single controller. In the first part, you simulate
the entire trajectory. In the second part you compute the motor commands as a function of this simulated trajectory. The thing is...
you don't necessarily have to divide it as so. You could say shorten the horizon of the simulated trajectory and run it on the low-level
side. I imagine there might be benefits to that because we have less issues of 'communicating' the data between two computers AND your state
estimator is always going to give the best estimate onboard (where the IMU is. yes mocap is offboard but we can more afford a slower position
update that an attitude update. Just try flying a quadcopter with a pure mocap and no IMU). Ok anyway I digress. What I'm trying to get at
is that whatever low-level controller is supposed to do... it has to do its stuff efficiently. So #1, that piece has to be coded in simple
and efficient forms. #2 we need to kill unnecessary apps that are running on the px4. Given that we do away with the original pos->att->rate
piece... we really don't need their apps in the drone. And neither do we need stuff like navigator while we're at that. Long story short... we
need our own cmake build profile where we are especially careful with what apps we launch at boot.

Plan
----
Ok enough with my rambling on what I hope to reach in the long horizon. Here's where I try to flesh out what I have already done and plan to do.

1) [DONE] Swap out the pos->att->rate controllers in firmware with the low level controller on mc builds. For now, I subscribe to an existing
uORB topic (rc_channels) just to test. 
2) [Kinda done] Custom boot to clean up unnecessary apps. I currently do this using a separate make file: px4_fmu_v4_direct (i think that's the name?)
and then I use a file in the sd card to stop mc_rate_controller. I'll work on a proper config later on.
3) [doing now]. Define a custom uORB topic to be published from a companion computer. For now... I'll just use the diff. flat spline as my default
traj message form. It's smaller than the iLQR packet (i think?) and it'll let me figure out what kind of bandwidth/latency constraints we will have.
We might need to use the FastRTPS bridge if this turns out to be a real bottleneck. PepMS over in the px4 slack has some solutions to this. Will look there
when we get there. But for now, we'll just stream the polynomial out. Or if that's too slow, we can load it in prior to a flight. My guess is we can
get away with that if trajectories are short.

Ah yes... so testing trajectories. FOR NOW... the only trajectories I will care about is a takeoff->hover->land and a takeoff->hover->line->land. There's
a whole can of worms to uncover in terms of parameter estimation and vibration dampening effects. I'll worry about that later. Test platform...
to give us the best shot... is a pixracer+upboard combo. This lets us run full ubuntu 18 onboard in a compact package. I won't try streaming the traj
packet over wifi just yet. Gonna use a wired connection to give it the best chance.

4) tidy up mc_direct_control and the custom uORB message so that it can be easily changed between diff. flat approach and iLQR approach. this should be 
a good test of generality. I worry though as traj is really going to be very different. Can it really be generalized that way? Not sure. 