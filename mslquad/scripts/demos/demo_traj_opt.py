# Description: 
#   Demo of minimal snap trajectory optimization
#   without optimizing section time duration
#   For time duration optimization, see:
#   demo_traj_opt_time.py
# Author: 
#   Zijian Wang, Stanford University
# Email: g
#   zjwang@stanford.edu

import numpy as np
from path.objdyn import LoadSim
from path.visual import VizTool, TrajPlotter
from path.robot import Thruster
from path.trajectory import PolynomialTraj, PolynomialTrajOpt
from path.waypoint import Keyframes, KeyframesPool                 

## define quadrotor and load
load = LoadSim()
quad = Thruster(load, 0.0, 0.0, 0.2, 1.0) 
viz = VizTool(load, [quad])

## define waypoints (retrieve one from the pre-defined library)
kfpool = KeyframesPool()  
kf = kfpool.get_keyframes(name = '003')

## plan trajectory using regular polynomial fit and 
## optimization-based approach to minimize snap
traj_opt = PolynomialTrajOpt(load, kf.wps, kf.ts)

## Alternatively, can retrieve a pre-optimized trajectory
# traj_opt = PolynomialTrajOpt()
# traj_opt.load_npz("saved_trajectory/traj.npz")

# plot the trajectory in 3D, and  reference force/moment input
tp = TrajPlotter()
tp.plot_traj(traj_opt, 'r-')
tp.plot_ref_stt_in(traj_opt, '-')
tp.show()

# fly the planned trajectory using a single quadrotor
tf = sum(kf.ts)
for i in range(int(tf/load.dT)):
    print "sim: ", i
    if i == 0:
        continue
    t = i * load.dT # simulation time

    fout = traj_opt.get_flatout(t)
    r_acc = fout[0:3, 2].reshape((3,1))
    r_euler, r_wb, r_pos, r_vel, r_fz, r_tau, r_R = \
            traj_opt.get_ref_state_input(t, fout)

    # control by one quadrotor
    tol_fz, tol_tau = quad.se3_ctrl(r_euler, r_wb, r_pos, r_vel, r_acc, r_R)

    # object dynamics    
    load.step_using_tol_ft(tol_fz, tol_tau)

viz.animation()
viz.plot_traj_history()
viz.plot_force_torque(traj_opt)