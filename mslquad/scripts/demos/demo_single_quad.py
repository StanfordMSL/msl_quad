# Description: 
#   Test a single quad flying a trajectory using SE(3) controller
# Author: 
#   Zijian Wang, Stanford University
# Email: 
#   zjwang@stanford.edu

import numpy as np
from quadsmanip.objdyn import LoadSim
from quadsmanip.visual import VizTool
from quadsmanip.robot import Thruster
from quadsmanip.trajectory import PolynomialTrajOpt  
from quadsmanip.waypoint import Keyframes, KeyframesPool

# create load
load = LoadSim()

# creat trajectory
kfpool = KeyframesPool()
kf = kfpool.get_keyframes(name = '002')
traj = PolynomialTrajOpt(load, kf.wps, kf.ts) 

# create robot
quad = Thruster(load, 0.0, 0.0, 0.2, 1.0) 

# set up visualization
viz = VizTool(load, [quad])

## begin simulation
done_viz = False
try:
    for i in range(int(sum(kf.ts)/load.dT)):
        if i%100 == 0:
            print "sim step: ", i
        if i == 0:
            continue
        t = i * load.dT # simulation time
        tol_fz = 0.0
        tol_tau = np.matrix('0.0; 0.0; 0.0')

        fout = traj.get_flatout(t)
        r_acc = fout[0:3, 2].reshape((3,1))
        r_euler, r_wb, r_pos, r_vel, r_fz, r_tau, r_R = \
                traj.get_ref_state_input(t, fout)

        ## original se(3) controller
        tol_fz, tol_tau = quad.se3_ctrl(r_euler, r_wb, r_pos, r_vel, r_acc, r_R)
        
        ## alternatively, can use the following more advanced se(3) controller
        ## smart se(3) controller determines yaw_dot online to constrain omega_z_des = 0
        # r_jer = fout[0:3, 3].reshape((3,1))
        # tol_fz, tol_tau = quad.se3_ctrl_online_yaw(
        #        r_euler, r_wb, r_pos, r_vel, r_acc, r_R, r_jer=r_jer)

        # object dynamics    
        load.step_using_tol_ft(tol_fz, tol_tau)

except:
    viz.animation()
    viz.plot_traj_history()
    done_viz = True

if not done_viz:
    print "Start animation..."
    viz.animation()
    viz.plot_traj_history()