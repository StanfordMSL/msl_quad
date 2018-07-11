# Description: Trajectory planning
# Author: Zijian Wang, Stanford University
# Date: Jul 18, 2017
# Email: zjwang@stanford.edu

import numpy as np
import diff
import cvxpy as cvx
import copy
import math
import pickle
import cvxtraj

def cvxgen_traj_solve(Q, wp, tf, cont, final_cons, n_sec):
    """
    Solve trajectory optimization using cvxgen.
    Currently, cvxgen takes in params for 8 sections.
    If need less than 8 sections, set corresponding params to zero.
    Inputs:
        Q: 2D list. Each element is a list with 16 numbers
        wp: waypoints, 1D list
        tf: time series, 2D list. Each element has size 8
        cont: continuity coefficients, 2d list.
        final_cons: final state constraint. 2d list. However only store
                    coefficients of the last section since other sections
                    have all zero coefficients
        n_sec: specify the number of total sections
               will be used to set parameters of the final constraint
    """
    Qs = [[0]*16] * 8
    for i in range(len(Q)):
        Qs[i] = Q[i]

    wps = [0] * 9
    for i in range(len(wp)):
        wps[i] = wp[i]

    ts = [[0]*8] * 8
    for i in range(len(tf)):
        ts[i] = tf[i]

    conts = [[0]*8]*(35) # 8x5x7
    for i in range(len(cont)):
        conts[i] = cont[i]

    final = [[0]*8]*24 # final state constraint
                       # all zero except the actual final section
    final[4*(n_sec-3)] = final_cons[0]
    final[4*(n_sec-3)+1] = final_cons[1]
    final[4*(n_sec-3)+2] = final_cons[2]
    final[4*(n_sec-3)+3] = final_cons[3]

    ret = cvxtraj.solve_traj(
        Qs[0], Qs[1], Qs[2], Qs[3], Qs[4], Qs[5], Qs[6], Qs[7], 
        wps,
        ts[0], ts[1], ts[2], ts[3], ts[4], ts[5], ts[6], ts[7], 
        final[0], final[1], final[2], final[3], 
        final[4], final[5], final[6], final[7], 
        final[8], final[9], final[10], final[11], 
        final[12], final[13], final[14], final[15], 
        final[16], final[17], final[18], final[19], 
        final[20], final[21], final[22], final[23], 
        conts[0], conts[1], conts[2], conts[3], conts[4], 
        conts[5], conts[6], conts[7], conts[8], conts[9], 
        conts[10], conts[11], conts[12], conts[13], conts[14], 
        conts[15], conts[16], conts[17], conts[18], conts[19], 
        conts[20], conts[21], conts[22], conts[23], conts[24], 
        conts[25], conts[26], conts[27], conts[28], conts[29], 
        conts[30], conts[31], conts[32], conts[33], conts[34]
    )

    c1,c2,c3,c4,c5,c6,c7,c8,converged = ret
    assert(converged == 1)

    retc = np.matrix([
        c1,c2,c3,c4,c5,c6,c7,c8
    ])
    retc = retc[0:n_sec, :]

    return retc

class Traj:
    """
    Base class for any trajectory. Encapsulate common operation such as getting
    reference state/input using differential flatness given flat trajectory
    """
    def __init__(self, load):
        self.M = load.M # mass of the load, useful in computing ref input by diff flatness

        # params that needs to be feed into differential flatness computation
        self.J = (load.J[0,0], load.J[1,1], load.J[2,2], load.J[0,1], load.J[1,2], load.J[0,2])
        self.Jinv = (load.Jinv[0,0], load.Jinv[1,1], load.Jinv[2,2], load.Jinv[0,1], load.Jinv[1,2], load.Jinv[0,2])

    def save2file(self, filename=None):
        if filename is None:
            filename = "default"
        pickle.dump(self, open( "log/traj/" + filename + ".p", "wb"))

    def time_basis(self, t, order):
        """
        return [1, t, t^2, t^3, ...]
        """
        return np.array([t ** i for i in range(order+1)])

    def get_ref_state_input(self, t, fout):
        """
        given a flat output trajectory, generate reference states and inputs using differential flatness
        """
        sig = tuple(fout[:,0])
        sigd1 = tuple(fout[:,1])
        sigd2 = tuple(fout[:,2])
        sigd3 = tuple(fout[:,3])
        sigd4 = tuple(fout[:,4])
        
        euler, wb, pos, vel, fz, tau, R = \
            diff.diff2SttIn( self.M, self.J, self.Jinv, sig, sigd1, sigd2, sigd3, sigd4 )
        
        # convert tuple output to np matrix
        r_euler = np.matrix([ [euler[0]], [euler[1]], [euler[2]] ])
        r_wb = np.matrix([ [wb[0]], [wb[1]], [wb[2]] ])
        r_pos = np.matrix([ [pos[0]], [pos[1]], [pos[2]] ])
        r_vel = np.matrix([ [vel[0]], [vel[1]], [vel[2]] ])
        r_fz = fz
        r_tau = np.matrix([ [tau[0]], [tau[1]], [tau[2]] ])
        r_R = np.matrix([ [R[0], R[1], R[2]], [R[3], R[4], R[5]], [R[6], R[7], R[8]] ])

        return r_euler, r_wb, r_pos, r_vel, r_fz, r_tau, r_R
    

class PolynomialTraj( Traj ):
    """
    trajectory design using time polynomial
    currently, 5th-order is used so that there are 6 coefficients, which can be uniquely
    determined given two endpoints and their vel and acc
    """
    def __init__(self, load=None, wps=None, ts=None):
        """
        inputs:
            wps: N x 4 x 3, all the waypoints
            ts: a list. the duration of each section. there are only (N-1) sections/durations
        """
        Traj.__init__(self, load)
        self.wps = wps
        self.ts = ts # duration of sections
        self.n_sec = len(ts) # number of sections in the trajectory
        
        self.t_table = [0.0] # total time schedule, starting w/ 0
        for i in range(0,len(ts)):
            self.t_table.append(self.t_table[i] + ts[i])

        # main result of polynomial fit, multiply F matrix by time basis to get flat output
        self.Fx = np.zeros((self.n_sec, 5, 8)) 
        self.Fy = np.zeros((self.n_sec, 5, 8))
        self.Fz = np.zeros((self.n_sec, 5, 8))
        self.Fpsi = np.zeros((self.n_sec, 5, 8))

    def update_t_table(self):
        self.t_table = [0.0] # total time schedule, starting w/ 0
        for i in range(0,len(self.ts)):
            self.t_table.append(self.t_table[i] + self.ts[i])

    def get_section_idx(self, t):
        """
        determine with section is it in given t
        """
        idx = np.searchsorted(self.t_table, t)
        if idx > 0:
            idx = idx - 1
        return idx

    def get_F_mat(self, coeff):
        """
        F * tt will give the 4x5 flat output, where tt is the time basis
        """
        return np.array([
            coeff, # sigma
            [coeff[1], 2.0*coeff[2], 3.0*coeff[3], 4.0*coeff[4], 5.0*coeff[5], 6.0*coeff[6], 7.0*coeff[7], 0.0],  # 1-st derivative
            [2.0*coeff[2], 6.0*coeff[3], 12.0*coeff[4], 20.0*coeff[5], 30.0*coeff[6], 42.0*coeff[7], 0.0, 0.0], # 2-nd derivative
            [6.0*coeff[3], 24.0*coeff[4], 60.0*coeff[5], 120.0*coeff[6], 210.0*coeff[7], 0.0, 0.0, 0.0], # 3-rd derivative
            [24.0*coeff[4], 120.0*coeff[5], 360.0*coeff[6], 840.0*coeff[7], 0.0, 0.0, 0.0, 0.0] # 4-th derivative            
        ])

    def get_flatout(self, t, order = 5):
        """
        order is the order of the polynomial
        """
        idx_sec = self.get_section_idx(t)
        t_start = self.t_table[idx_sec]
        t_in_sec = t - t_start # relative time within the section

        tt = self.time_basis(t_in_sec, order)
        flat_x = np.matmul(self.Fx[idx_sec,:,:], tt)
        flat_y = np.matmul(self.Fy[idx_sec,:,:], tt)
        flat_z = np.matmul(self.Fz[idx_sec,:,:], tt)
        flat_psi = np.matmul(self.Fpsi[idx_sec,:,:], tt)
        return np.vstack(
            (flat_x, flat_y, flat_z, flat_psi)
        )

class PolynomialTrajOpt( PolynomialTraj ):
    """
    Time polynomial traj planning using convex optimization 
    (default 7th order polynomial)
    only (x,y,z,psi) needs to be specified for each waypoint, 
    vel, acc, jerk, snap will be determined by solving a QP  
    assumed the initial vel and acc, final vel and acc are zero
    """
    def __init__(self, load=None, wps=None, ts=None, use_cvxgen=True):
        """
        inputs:
            wps: N x 4 x 3, same as PolynomialTraj, 
                 however only position will be used, vel and acc will be ignored
            ts: a list. the duration of each section. there are only (N-1) sections/durations
            use_cvxgen: set to True to use cvxgen (~10x faster), otherwise use cvxpy. 
        """
        if load is None or wps is None or ts is None:
            return # must use load_npz later to load params
        assert(wps.shape[0]>=4) # need at least 3 waypoints for the problem to be feasible
        PolynomialTraj.__init__(self, load, wps, ts)
        self.use_cvxgen = use_cvxgen
        if wps.shape[0]-1 > 8: # existing cvxgen code only supports 8 sections (9 waypoints).  
            self.use_cvxgen = False
        self.optimize_all()

    def save2npz(self, filename):
        np.savez(filename, Fx=self.Fx, Fy=self.Fy, Fz=self.Fz, Fpsi = self.Fpsi,
                    ts=self.ts, t_table=self.t_table, 
                    M=self.M, J=self.J, Jinv=self.Jinv)

    def load_npz(self, filename):
        npzfile = np.load(filename)
        self.Fx = npzfile['Fx']
        self.Fy = npzfile['Fy']
        self.Fz = npzfile['Fz']
        self.Fpsi = npzfile['Fpsi']
        self.ts = npzfile['ts'].tolist()
        self.t_table = npzfile['t_table'].tolist()
        self.M = npzfile['M']
        self.J = tuple([e for e in npzfile['J']]) # convert to tuple
        self.Jinv = tuple([e for e in npzfile['Jinv']])

    def optimize_all(self, ts = None):
        """
        solve polynomial coefficients for each dimension independently
        results:
            F matrices on x, y, z, psi directions
        outputs:
            None
        """
        if ts is None:
            ts = self.ts

        wp_x = []
        wp_y = []
        wp_z = []
        wp_psi = []
        for i in range(len(self.wps)):
            wp_x += [self.wps[i,0,0]]
            wp_y += [self.wps[i,1,0]]
            wp_z += [self.wps[i,2,0]]
            wp_psi += [self.wps[i,3,0]]

        # get coefficients, each row corresponds to a trajectory section
        # c is a numpy matrix, shape: (# of sections) x (# of poly order + 1)
        cx = self.optimize_1d(wp_x, ts)
        cy = self.optimize_1d(wp_y, ts)
        cz = self.optimize_1d(wp_z, ts)
        cpsi = self.optimize_1d(wp_psi, ts)

        for i in range(self.n_sec):
            self.Fx[i,:,:] = self.get_F_mat( np.asarray(cx[i,:]).reshape(-1) )
            self.Fy[i,:,:] = self.get_F_mat( np.asarray(cy[i,:]).reshape(-1) )
            self.Fz[i,:,:] = self.get_F_mat( np.asarray(cz[i,:]).reshape(-1) )
            self.Fpsi[i,:,:] = self.get_F_mat( np.asarray(cpsi[i,:]).reshape(-1) )

    def optimize_1d(self, waypoint, ts):
        """
        solve the coefficients for 1 dimension by solving QP, 
        default 7th order polynomial
        the objective is the integration of fourth derivative of 
        position (snap) squared over the trajectory section
        note:
            assumed the time of each section is shifted to start at zero
        outputs:
            c: numpy matrix, shape: (# of sections) x (# of poly order + 1)
        """
        n_section = len(waypoint)-1 # total number of sections in the trajectory

        if not self.use_cvxgen: # use cvxpy
            c = cvx.Variable(n_section, 8)
            cost = 0.0
            constraints = []
            # first section constraints
            constraints += [
                c[0,1] == 0.0, # initial velocity is zero
                2.0*c[0,2] == 0.0, # initial acceleration is zero
                6.0*c[0,3] == 0.0, # initial jerk
                24.0*c[0,4] == 0.0, # initial snap
            ]
            for i in range(n_section):
                tf = ts[i]

                Q = np.matrix([
                    [576.0 * tf,        1440.0 * (tf**2 ),  2880.0 * (tf**3), 5040.0*(tf**4)],
                    [1440.0 * (tf**2),  4800.0 * (tf**3),   10800.0 * (tf**4), 20160.0*(tf**5)],
                    [2880.0 * (tf**3),  10800.0 * (tf**4),  25920.0 * (tf**5), 50400.0*(tf**6)],
                    [5040.0*(tf**4), 20160.0*(tf**5), 50400.0*(tf**6), 100800*(tf**7)],
                ]) 

                # integration of snap squared, which has analytical solution
                cost += cvx.quad_form(c[i, 4:].T, Q) 
                
                tt0 = self.time_basis(0.0, 7) # assumed the time of each section is shifted to start at zero
                ttf = self.time_basis(tf, 7)
                constraints += [ # waypoint position constraint
                    c[i,:] * tt0.T == waypoint[i],
                    c[i,:] * ttf.T == waypoint[i+1],
                ]
                if i < n_section - 1: # not the last section
                    constraints += [ # transition continuity constraint
                        c[i,1] + 2.0*c[i,2]*tf + 3.0*c[i,3]*(tf**2) + 4.0*c[i,4]*(tf**3) + 5.0*c[i,5]*(tf**4) + 6.0*c[i,6]*(tf**5) + 7.0*c[i,7]*(tf**6)== c[i+1,1], # velocity
                        2.0*c[i,2] + 6.0*c[i,3]*tf + 12.0*c[i,4]*(tf**2) + 20.0*c[i,5]*(tf**3) +30.0*c[i,6]*(tf**4) + 42.0*c[i,7]*(tf**5) == 2.0*c[i+1,2], # acceleration
                        6.0*c[i,3] + 24.0*c[i,4]*tf + 60.0*c[i,5]*(tf**2) + 120.0*c[i,6]*(tf**3) + 210.0*c[i,7]*(tf**4) == 6.0*c[i+1,3], # jerk
                        24.0*c[i,4] + 120.0*c[i,5]*tf + 360.0*c[i,6]*(tf**2) + 840.0*c[i,7]*(tf**3) == 24.0*c[i+1,4], # snap
                        120.0*c[i,5] + 720.0*c[i,6]*tf + 2520.0*c[i,7]*(tf**2) == 120.0*c[i+1,5], # derivative of snap, to ensure smooth reference torque
                    ]  
            # last section constraints
            tf = ts[n_section-1]
            constraints += [
                c[n_section-1,1] + 2.0*c[n_section-1,2]*tf + 3.0*c[n_section-1,3]*(tf**2) + 4.0*c[n_section-1,4]*(tf**3) + 5.0*c[n_section-1,5]*(tf**4) + 6.0*c[n_section-1,6]*(tf**5) + 7.0*c[n_section-1,7]*(tf**6) == 0.0, # velocity
                2.0*c[n_section-1,2] + 6.0*c[n_section-1,3]*tf + 12.0*c[n_section-1,4]*(tf**2) + 20.0*c[n_section-1,5]*(tf**3) + 30.0*c[n_section-1,6]*(tf**4) + 42.0*c[n_section-1,7]*(tf**5) == 0.0, # acceleration
                6.0*c[n_section-1,3] + 24.0*c[n_section-1,4]*tf + 60.0*c[n_section-1,5]*(tf**2) + 120.0*c[n_section-1,6]*(tf**3) + 210.0*c[n_section-1,7]*(tf**4) == 0.0, # jerk
                24.0*c[n_section-1,4] + 120.0*c[n_section-1,5]*tf + 360.0*c[n_section-1,6]*(tf**2) + 840.0*c[n_section-1,7]*(tf**3) == 0.0 # snap
            ]

            prob = cvx.Problem(cvx.Minimize(cost), constraints)
            prob.solve(solver=cvx.CVXOPT, abstol=1e-1) # the objective function (integration of snap) is very large
                                                    # therefore needs a "relatively large" abstol here, otherwise very hard to achieve accuracy

            # print "status:", prob.status
            # print "optimal var", c.value
            if prob.status != 'optimal':
                print "ts = ", ts
                print "[Warning] cvxpy ", prob.status
            return c.value

        else: # use cvxgen 
            Qlist = []
            tfs = []
            final_cons = [] # coefficients of final state constraint 
            cont = [] # coefficients of continuity constraint

            for i in range(n_section):
                tf = ts[i]     
                ttf = self.time_basis(tf, 7)
                tfs.append(ttf)
                Q = [576.0 * tf,        1440.0 * (tf**2 ),  2880.0 * (tf**3), 5040.0*(tf**4),
                    1440.0 * (tf**2),  4800.0 * (tf**3),   10800.0 * (tf**4), 20160.0*(tf**5),
                    2880.0 * (tf**3),  10800.0 * (tf**4),  25920.0 * (tf**5), 50400.0*(tf**6),
                    5040.0*(tf**4), 20160.0*(tf**5), 50400.0*(tf**6), 100800*(tf**7)]
                Qlist.append(Q)

                if i < n_section - 1:
                    cont.append(
                        [0., 1., 2.0*tf, 3.0*(tf**2), 4.0*(tf**3), 5.0*(tf**4), 6.0*(tf**5), 7.0*(tf**6)])
                    cont.append(
                        [0., 0., 2.0, 6.0*tf, 12.0*(tf**2), 20.0*(tf**3), 30.0*(tf**4), 42.0*(tf**5)])
                    cont.append(
                        [0., 0., 0., 6.0, 24.0*tf, 60.0*(tf**2), 120.0*(tf**3), 210.0*(tf**4)])
                    cont.append(
                        [0., 0., 0., 0., 24.0, 120.0*tf, 360.0*(tf**2), 840.0*(tf**3)])
                    cont.append(
                        [0., 0., 0., 0., 0., 120.0, 720.0*tf, 2520.0*(tf**2)])

            # last section constraints
            tf = ts[n_section-1]
            final_cons.append(
                [0., 1., 2.0*tf, 3.0*(tf**2), 4.0*(tf**3), 5.0*(tf**4), 6.0*(tf**5), 7.0*(tf**6)])
            final_cons.append(
                [0., 0., 2.0, 6.0*tf, 12.0*(tf**2), 20.0*(tf**3), 30.0*(tf**4), 42.0*(tf**5)])
            final_cons.append(
                [0., 0., 0., 6.0, 24.0*tf, 60.0*(tf**2), 120.0*(tf**3), 210.0*(tf**4)])
            final_cons.append(
                [0., 0., 0., 0., 24.0, 120.0*tf, 360.0*(tf**2), 840.0*(tf**3)])

            return cvxgen_traj_solve(Qlist, waypoint, tfs, cont, final_cons, n_section)


    def get_flatout(self, t):
        return PolynomialTraj.get_flatout(self, t, 7)


class PolynomialTrajOptTime( PolynomialTrajOpt ):
    """
    no need to specify the duration of the sections.
    durations will be optimized in order to: 
        (a) minimize total time, 
        (b) make sure the max reference force/torque input is less than the specified bound
    used cyclic coordinate descent on section durations (gradient-free)
    used exterior point method to deal with the input constraint
    """
    def __init__(self, 
                load, wps, tmax, fz_eq, fz_thres, 
                taux_thres, tauy_thres, tauz_thres, 
                n_cycle=10, n_line_search=15):
        """
        Inputs:
            tmax: maximum duration per section, used in golden section search
            fz_eq: total thrust in hover equilibrium
            fz_thres: maximum +/- thrust around fz_eq
            tau_thres: maximum moment
            n_cycle: max # of cycles for coordinate descent
            n_line_search: # of function evaluations per line search
        """
        TMAX = tmax
        self.load = load
        self.fz_eq = fz_eq
        self.fz_thres = fz_thres
        self.taux_thres = taux_thres
        self.tauy_thres = tauy_thres
        self.tauz_thres = tauz_thres

        self.n_sec = wps.shape[0] - 1 # total number of sections
        self.ts = [TMAX] * self.n_sec # initialize the durations
        PolynomialTraj.__init__(self, load, wps, self.ts)

        # find the optimal section durations using cyclic coordinate descent 
        # and exterior point method
        print "Begin optimizing section durations ..."
        best_ts = None
        best_tsum = float('Inf')
        best_violate = float('Inf')
        best_feasible = False
        prev_best_ts = None
        for i in range(n_cycle):
            for j in range(self.n_sec): 
                topt0, topt1, v0, v1 = self.goldenSectionSearch(
                            sec_idx=j, tmax=TMAX, n=n_line_search, r=10.0**(i+1))
                is_feasible = False
                if(v0 <= 1e-3):
                    self.ts[j] = topt0
                    is_feasible = True
                elif (v1 <= 1e-3):
                    self.ts[j] = topt1
                    is_feasible = True
                else:
                    self.ts[j] = topt1 # if not feasible, prefer longer section duration
                                       # but keep is_feasible = False
                # update the best solution, if needed
                if(is_feasible): # new solution is feasible
                    if(not best_feasible):
                        best_feasible = True
                        best_ts = copy.copy(self.ts)
                        best_tsum = sum(self.ts)
                        best_violate = 0
                    else:
                        tsum = sum(self.ts)
                        if( tsum < best_tsum):
                            best_ts = copy.copy(self.ts)
                            best_tsum = tsum
                            best_violate = 0
                else: # new solution is not feasible
                    if(not best_feasible): # both new/best solutions are not feasible
                        if(v1 < best_violate):
                            tsum = sum(self.ts)
                            if(tsum < best_tsum):
                                best_ts = copy.copy(self.ts)
                                best_tsum = sum(best_ts)
                                best_violate = v1
                print "progress: ", i*self.n_sec+j, " / ", n_cycle*self.n_sec
                # print "is_feasible: ", is_feasible
            # check convergence
            if(best_feasible and (prev_best_ts == best_ts)):
                break
            prev_best_ts = best_ts
        # assign best solution found
        self.ts = best_ts
        print "Finished optimizing section durations"
        print "Best ts: ", self.ts
        print "Best violate = ", best_violate


        PolynomialTrajOpt.__init__(self, load, wps, self.ts)

    def time_input_cost(self, ts, r, check_time_step=0.1):
        """
        define the cost function, given the section durations
        the input constraint is relaxed by adding the violation to the cost (scaled by r)
        this is analogous to the exterior point method
        Inputs:
            ts: durations times for all sections
            r: penalty scale on constraint violation
            check_time_step: resolution for checking thrust/moments using differential flatness,
                             the trajectory is discretized by this time step, since the traj 
                             is very smooth, choosing a large time step should be fine in order
                             to speed up computation.
        """
        cost_time = sum(ts)
        
        # design the trajectory given the waypoints and ts
        temp_traj = PolynomialTrajOpt(self.load, self.wps, ts)

        # need to "enumerate" the entire trajectory to find out the amount of violation
        # this is because the analytical expressions for fz and tau are extremely complicated
        fz_max = 0.0
        taux_max = 0.0
        tauy_max = 0.0
        tauz_max = 0.0
        T = sum(ts) # total time of the trajectory
        for t in np.arange(0.0, T, check_time_step):
            fout = temp_traj.get_flatout(t)
            r_euler, r_wb, r_pos, r_vel, r_fz, r_tau, r_R = temp_traj.get_ref_state_input(t, fout)
            if abs(r_fz - self.fz_eq) > fz_max:
                fz_max = abs(r_fz - self.fz_eq)
            if abs(r_tau[0,0]) > taux_max:
                taux_max = abs(r_tau[0,0])
            if abs(r_tau[1,0]) > tauy_max:
                tauy_max = abs(r_tau[1,0])
            if abs(r_tau[2,0]) > tauz_max:
                tauz_max = abs(r_tau[2,0])

        # cost will be normalized
        cost_fz = max( (fz_max - self.fz_thres) / self.fz_thres, 0.0)
        cost_taux = max( (taux_max - self.taux_thres) / self.taux_thres, 0.0 )
        cost_tauy = max( (tauy_max - self.tauy_thres) / self.tauy_thres, 0.0 )
        cost_tauz = max( (tauz_max - self.tauz_thres) / self.tauz_thres, 0.0 )

        violate_cons = cost_fz + cost_taux + cost_tauy + cost_tauz

        # final cost is total time + violation of constraints
        return cost_time + r * (cost_fz + cost_taux + cost_tauy + cost_tauz), violate_cons

    def goldenSectionSearch(self, sec_idx, tmax, n, r):
        """
        golden search (1D)
        optimize the duration for 1 section each time in one coordinate descent
        """
        violate1 = None
        violate4 = None

        al = (math.sqrt(5) - 1) / 2
        x1 = 0.0
        x4 = tmax
        x3 = al * x4 + (1 - al) * x1

        ts = copy.copy(self.ts)
        ts[sec_idx] = x3
        f3, violate3 = self.time_input_cost(ts=ts, r=r)
        for i in range(n-1):
            x2 = al * x1 + (1 - al) * x4
            ts[sec_idx] = x2
            f2, violate2 = self.time_input_cost(ts=ts, r=r)
            if f2 < f3:
                x4 = x3
                violate4 = violate3
                x3 = x2
                f3 = f2
                violate3 = violate2
            else:
                x1 = x4
                violate1 = violate4
                x4 = x2
                f4 = f2
                violate4 = violate2
        
        # resolve unevaluated constraint violance
        if (violate1 is None):
            ts[sec_idx] = x1
            _, violate1 = self.time_input_cost(ts=ts, r=r)
        if (violate4 is None):
            ts[sec_idx] = x4
            _, violate4 = self.time_input_cost(ts=ts, r=r)

        if x1 < x4:
            return (x1, x4, violate1, violate4)
        else:
            return (x4, x1, violate4, violate1)