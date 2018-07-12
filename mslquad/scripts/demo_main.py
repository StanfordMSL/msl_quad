# Description: 
#   Cooperative object transport with 8 quadrotors without communication
#   Demonstrate the controller proposed in the ICRA 18 paper
# Author: 
#   Zijian Wang, Stanford University
# Email: 
#   zjwang@stanford.edu

import numpy as np
from path.objdyn import LoadSim
from path.visual import VizTool, TrajPlotter
from path.robot import Thruster
from path.trajectory import PolynomialTrajOpt, PolynomialTrajOptTime
from path.waypoint import Keyframes, KeyframesPool
import matplotlib
#matplotlib.use('TkAgg')

## create payload with default parameters
load = LoadSim()

## initialize quadrotors
quads = []
quad_pos = [
    [0.5, 0.0],
    [0.0, 0.5],
    [-0.5, 0.0],
    [0.0, -0.5],
    [0.4, 0.4],
    [-0.4, 0.4],
    [-0.4, -0.4],
    [0.4, -0.4]
]
NUM_QUAD = 8.0
for i in range(int(NUM_QUAD)):
    # add random perturbation on the robots' attachment positions
    xnoise = np.random.uniform(low=-0.05, high=0.05)
    ynoise = np.random.uniform(low=-0.05, high=0.05)
    quad = Thruster(load, quad_pos[i][0]+xnoise, quad_pos[i][1]+ynoise, 0.2, NUM_QUAD, i)
    quads.append(quad)

## initialize visualization
viz = VizTool(load, quads)

## optimize a trajectory from scratch
# kfpool = KeyframesPool()
# kf = kfpool.get_keyframes(name = '003')
# traj_opt_time = PolynomialTrajOptTime(
#       load, kf.wps, tmax=5.3, fz_eq=-19.8, 
#       fz_thres=0.5, taux_thres=0.01, tauy_thres=0.01, tauz_thres=0.01, 
#       n_cycle = 5)

## retrieve a already optimized trajectory
traj_opt_time = PolynomialTrajOpt()
traj_opt_time.load_npz("saved_trajectory/traj_time_opt.npz")


##### begin simulation #####

## set initial pose error
# load.set_state(pos = np.matrix("0.07; -0.07; 0.0"))

final_hover_time = 2.0
done_viz = False
try:
    for i in range(int(sum(traj_opt_time.ts)/load.dT)+int(final_hover_time/load.dT)):
        if i%100 == 0:
            print "sim step: ", i
        if i == 0:
            continue
        t = i * load.dT # simulation time
        tol_fz = 0.0
        tol_tau = np.matrix('0.0; 0.0; 0.0')
        if (i < int(sum(traj_opt_time.ts)/load.dT)):
            fout = traj_opt_time.get_flatout(t)
            r_acc = fout[0:3, 2].reshape((3,1))
            r_jer = fout[0:3, 3].reshape((3,1))
            r_euler, r_wb, r_pos, r_vel, r_fz, r_tau, r_R = \
                    traj_opt_time.get_ref_state_input(t, fout)
        else: # final hovering position
            r_euler = np.matrix("0.0;0.0;0.0")
            r_wb = np.matrix("0.0;0.0;0.0")
            r_pos = np.matrix("0.0;7.0;-1.0")
            r_vel = np.matrix("0.0;0.0;0.0")
            r_fz = -19.8
            r_tau = np.matrix("0.0;0.0;0.0")
        
        # decentralized control
        for qd in quads:
            # each quad independently computes its own wrench
            # without communicating with other robots
            fz, tau = qd.local_wrench(r_euler, r_wb, r_pos, r_vel, r_acc, r_R, r_jer=r_jer)
            tol_fz += fz
            tol_tau += np.copy(tau)

        # print "Sum wrench: ", tol_fz, tol_tau[0], tol_tau[1], tol_tau[2]

        # object dynamics    
        load.step_using_tol_ft(tol_fz, tol_tau)

except Exception as e:
    print e.message
    viz.animation()
    viz.plot_traj_history()
    done_viz = True

if not done_viz:
    print "Start animation..."
    viz.animation()
    viz.plot_traj_history()

