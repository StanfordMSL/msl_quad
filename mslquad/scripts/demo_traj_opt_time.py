# Description: 
#   Trajectory planning under input constraint
#   Generate a trajectory with feasible thrust and moments
#   that does not saturate the motors
#   More details in the ICRA 18 paper
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

load = LoadSim()
quad = Thruster(load, 0.0, 0.0, 0.2, 1.0) 
viz = VizTool(load, [quad])
kfpool = KeyframesPool()
kf = kfpool.get_keyframes(name = '003') # retrieve waypoints

## For comparison, generate trajectory using two methods
## First, generate trajectory by specifying waypoint positions and section durations
traj_opt = PolynomialTrajOpt(load, kf.wps, kf.ts)
# traj_opt.save2npz("saved_trajectory/traj.npz") # save trajectory if needed
## Alternatively, retrieve saved trajectory
# traj_opt = PolynomialTrajOpt()
# traj_opt.load_npz("saved_trajectory/traj.npz")

## Second, no need to specify durations
## Generate trajector such that the thrust/moments are below
## the given limits, leveraging differential flatness theory
traj_opt_time = PolynomialTrajOptTime(
        load, kf.wps, tmax=5.3, fz_eq=-19.8, fz_thres=0.5, 
        taux_thres=0.01, tauy_thres=0.01, tauz_thres=0.01, 
        n_cycle=5, n_line_search=15)
T = sum(traj_opt_time.ts) 
print T
for t in np.arange(0.0, T, 0.01):
    print traj_opt_time.get_flatout(t)
    print('next\n')

# traj_opt_time.save2npz("saved_trajectory/traj_time_opt.npz")
## Alternatively, retrieve saved trajectory
# traj_opt_time = PolynomialTrajOpt()
# traj_opt_time.load_npz("saved_trajectory/traj_time_opt.npz")


# # plot and compare
# tp = TrajPlotter()
# tp.plot_traj(traj_opt, 'r--')
# tp.plot_traj(traj_opt_time, 'k-')
# tp.plot_ref_stt_in(traj_opt, '--')
# tp.plot_ref_stt_in(traj_opt_time, '-')
# tp.show()