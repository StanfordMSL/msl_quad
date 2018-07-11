# Description: Robot model
# Author: Zijian Wang, Stanford University
# Date: Jul 18, 2017
# Email: zjwang@stanford.edu

import numpy as np
import diff
import math
from cvxpy import *
import cvxse3 # cvxgen code for solving the distributed se(3) controller
import cvxse3comp

def cvxgen_se3_solve(W, fz_des, tau_des, fmin, fmax):
    """
    interface function to call the cvxgen code (in C)
    """
    # prepare input for cvxgen
    W_tup = (W[1,0], W[1,1], W[1,2], W[1,3],
             W[2,0], W[2,1], W[2,2], W[2,3],
             W[3,0], W[3,1], W[3,2], W[3,3]) # only need row 2~4 since row 1 is all ones
    w_des = (fz_des, tau_des[0,0], tau_des[1,0], tau_des[2,0])
    fmin_tup = (fmin, )
    fmax_tup = (fmax, )

    # solve by cvxgen
    f, converged = cvxse3.solve_se3(W_tup, w_des, fmin_tup, fmax_tup)

    if converged != 1:
        print "[Error]: cvxgen didn't converge."
        raise RuntimeError
    
    # assemble the final output
    ff = np.matrix([
        [f[0]],
        [f[1]],
        [f[2]],
        [f[3]]
    ])
    return ff

def cvxgen_se3_comp_solve(W, fz_des, tau_des, fmin, fmax, wy_lb, wy_ub):
    """
    interface function to call the cvxgen code (in C)
    """
    # prepare input for cvxgen
    W_tup = (W[1,0], W[1,1], W[1,2], W[1,3],
             W[2,0], W[2,1], W[2,2], W[2,3],
             W[3,0], W[3,1], W[3,2], W[3,3]) # only need row 2~4 since row 1 is all ones
    w_des = (fz_des, tau_des[0,0], tau_des[1,0], tau_des[2,0])
    fmin_tup = (fmin, )
    fmax_tup = (fmax, )
    wy_lb_tup = (wy_lb, )
    wy_ub_tup = (wy_ub, )

    # solve by cvxgen
    f, converged = cvxse3comp.solve_se3_comp(W_tup, w_des, fmin_tup, fmax_tup, wy_lb_tup, wy_ub_tup)

    if converged != 1:
        print "[Error]: cvxgen didn't converge."
        raise RuntimeError
    
    # assemble the final output
    ff = np.matrix([
        [f[0]],
        [f[1]],
        [f[2]],
        [f[3]]
    ])
    return ff


class Thruster:
    """
    Thruster model, could be a single propeller or a quadrotor
    """
    def __init__(self, load, X, Y, arm_len, tol_bot, id=0, 
                fmax_per_motor=0.9, KP=4.0, KV=6.0, KR=15.0, KW=8.0):
        """
        Inputs:
            load: class LoadSim object as defined in objdyn
            X: float number, attachment position of thruster/quadrotor in object's frame
            Y: float number, same as X
            arm_len: arm length of the quadrotor (nothing to do with the attachment location)
            tol_bot: total number of robots
            id: id number
            fmax_per_motor: self-explanatory
            KP, KV, KR, KW: se(3) controller gain for position, lin vel, attitude, angular vel
        """
        self.id = id
        # object that the thruster is attached to
        self.load = load
        self.tol_bot = tol_bot # total number of robots attached

        # thruster property
        self.X = float(X) # attachment position w.r.t. load
        self.Y = float(Y)
        self.D = math.sqrt(self.X**2 + self.Y**2) # distance to the COM of the object
        self.alpha = np.arctan2(self.Y, self.X) # angle of the attachment position, in [-pi, pi]
        self.arm_len = arm_len # length of the arm
        self.C = 0.02 # induced torque coeff. tau_z = C * f_z
        self.FMAX_PER_MOTOR = fmax_per_motor # max thrust force, per rotor, 
                            #since Z-down, we need to use -self.FMAX_PER_MOTOR in the optimization

        # SE(3) controller gain, default values
        self.KP = KP
        self.KV = KV
        self.KR = KR
        self.KW = KW

        # some matrix for mapping the motor force to final wrench applied to the object
        #                  x
        #                x x x
        #                  x        m1
        #         x        |    m4 x
        #       x x x  - - - - - x x x    -> x-axis
        #         x        |       x   m2
        #                  x       m3
        #                x x x 
        #                  x
        #                           (z-down)
        #                y-axis
        self.W_m2b = np.matrix([
            [1.0, 1.0, 1.0, 1.0],
            [-self.arm_len, 0.0, self.arm_len, 0.0],
            [0.0, -self.arm_len, 0.0, self.arm_len],
            [self.C, -self.C, self.C, -self.C]
        ]) # motor force to body wrench
        self.W_b2o = np.matrix([
            [1.0, 0.0, 0.0, 0.0],
            [self.Y, 1.0, 0.0, 0.0],
            [-self.X, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]) # body wrench to object 
        self.W_m2o = self.W_b2o * self.W_m2b # self.W_m2o * [f1, f2, f3, f4] is the total wrench to the object
        self.W_o2m = np.linalg.inv(self.W_m2o) # self.W_o2m * wrench_on_object is the four motor forces
        self.W_o2s = np.matrix([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, np.cos(self.alpha), np.sin(self.alpha), 0.0],
            [0.0, -np.sin(self.alpha), np.cos(self.alpha), 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]) # convert wrench in object frame to a special frame,in the result, 
           # we can control the second element, and want to minimize the thir element        
        self.W_m2s = self.W_o2s * self.W_m2o
        self.W_temp = np.copy(self.W_m2s) # temp matrix used in calculating pstar in the distributed controller
        self.W_temp[2,:] = -self.W_temp[2,:]

        # measured state of the object
        self.mea_euler = np.matrix([[0.0], [0.0], [0.0]])
        self.mea_wb = np.matrix([[0.0], [0.0], [0.0]])
        self.mea_pos = np.matrix([[0.0], [0.0], [0.0]])
        self.mea_vel = np.matrix([[0.0], [0.0], [0.0]])

        # desired altitude of the object
        self.des_euler = np.matrix([[0.0], [0.0], [0.0]])
        self.des_wb = np.matrix([[0.0], [0.0], [0.0]])
        self.des_pos = np.matrix([[0.0], [0.0], [0.0]])
        self.des_vel = np.matrix([[0.0], [0.0], [0.0]])
        self.des_yaw = 0. # desired yaw, determined online if using se3_ctrl_online_yaw
        self.pre_thrust = self.load.M * self.load.g

        # logging
        self.des_pos_hist = [] # history of desired position
        self.ref_fz_hist = [] # reference fz
        self.fz_hist = [] #actual fz
        self.ref_tau_hist = []
        self.tau_hist = []

    def setpoint(self, des_euler=None, des_wb=None, des_pos=None, des_vel=None):
        """
        set desired state of the load
        """
        if des_euler is not None:
            assert( des_euler.shape == (3,1) )
            self.des_euler = np.copy(des_euler)
        if des_wb is not None:    
            assert( des_wb.shape == (3,1) )
            self.des_wb = np.copy(des_wb)
        if des_pos is not None:
            assert( des_pos.shape == (3,1) )
            self.des_pos = np.copy(des_pos)
        if des_vel is not None:
            assert( des_vel.shape == (3,1) )
            self.des_vel = np.copy(des_vel)

    def sense(self):
        """
        noisy sensing
        """
        self.sense_groundtruth()
        self.mea_euler += 0.003 * np.random.normal(size = (3,1))
        self.mea_wb += 0.001 * np.random.normal(size = (3,1)) 
        self.mea_pos += 0.02 * np.random.normal(size = (3,1)) 
        self.mea_vel += 0.01 * np.random.normal(size = (3,1)) 
        self.mea_R = np.copy( self.load.get_rot_mat(self.mea_euler) )

    def sense_groundtruth(self):
        """
        noiseless sensing
        """
        self.mea_euler,\
        self.mea_wb,\
        self.mea_pos,\
        self.mea_vel =  self.load.get_state()
        self.mea_R = np.copy( self.load.get_rot_mat(self.mea_euler) )

    def set_controller_gain(self, kp, kv, kr, kw):
        self.KP = kp
        self.KV = kv
        self.KR = kr
        self.KW = kw

    def se3_ctrl(self, r_euler, r_wb, r_pos, r_vel, r_acc, r_R, r_fz = None, r_tau = None, noise_sensor=False):
        """
        test function, assuming only one quadrotor located at the C.O.M. of the object
        this implements the SE(3) controller in the Kumar paper
        """
        fz = 0
        tau = np.matrix('0.0; 0.0; 0.0')

        if noise_sensor:
            self.sense()
        else:
            self.sense_groundtruth()

        ep = self.mea_pos - r_pos
        ev = self.mea_vel - r_vel
        zw = np.matrix('0.0; 0.0; 1.0')
        Fdes = -self.KP * ep - self.KV * ev - self.load.M * self.load.g * zw + self.load.M * r_acc
        
        zb = self.mea_R[:,2]

        zb_des = - Fdes / np.linalg.norm(Fdes)
        xc_des = np.matrix([
            [np.cos(r_euler[2,0])],
            [np.sin(r_euler[2,0])],
            [0.0]
        ])
        zb_des_cross_xc_des = np.cross(zb_des, xc_des, axisa = 0, axisb = 0).reshape((3,1))
        mag_zb_des_cross_xc_des = np.linalg.norm(zb_des_cross_xc_des)
        yb_des = zb_des_cross_xc_des / mag_zb_des_cross_xc_des
        xb_des = np.cross(yb_des, zb_des, axisa = 0, axisb = 0).reshape((3,1))
        R_des = np.hstack((xb_des, yb_des, zb_des))

        fz = np.dot(Fdes.T, zb)[0,0]

        temp = (R_des.T * self.mea_R - self.mea_R.T * R_des)
        eR = 0.5 * np.matrix( [ [temp[2,1]], [temp[0,2]], [temp[1,0]] ] )
        ew = self.mea_wb - r_wb

        tau = -self.KR * eR - self.KW * ew

        # logging
        self.ref_fz_hist.append( np.copy(r_fz) )
        self.ref_tau_hist.append( np.copy(r_tau) )
        self.fz_hist.append( np.copy(fz) )
        self.tau_hist.append( np.copy(tau) )

        return fz, tau

    def se3_ctrl_online_yaw(self, r_euler, r_wb, r_pos, r_vel, r_acc, r_R, 
            r_fz = None, r_tau = None, noise_sensor=False, r_jer=None, log=True):
        """
        Improved SE(3) controller with smart tau_z control to enforce om_z_des = 0
        """
        fz = 0
        tau = np.matrix('0.0; 0.0; 0.0')

        if noise_sensor:
            self.sense()
        else:
            self.sense_groundtruth()

        ep = r_pos - self.mea_pos
        ev = r_vel - self.mea_vel
        zw = np.matrix('0.0; 0.0; 1.0')
        Fdes = self.KP * ep + self.KV * ev - self.load.M * self.load.g * zw + self.load.M * r_acc
        thrust_des = (Fdes.T*(self.mea_R*np.matrix('[0;0;-1]')))[0,0]
        zb = - Fdes
        zb_des = zb / np.linalg.norm(zb)
        yc_des = np.matrix([
            [-np.sin(self.des_yaw)],
            [np.cos(self.des_yaw)],
            [0.]
        ])
        xb = np.cross(yc_des, zb_des, axisa=0, axisb=0).reshape((3,1))
        xb_des = xb/np.linalg.norm(xb)
        yb_des = np.cross(zb_des, xb_des, axisa=0, axisb=0).reshape((3,1))
        R_des = np.hstack((xb_des, yb_des, zb_des))

        # get desired rates
        acc = np.matrix('0.0; 0.0; 1.0')*self.load.g - \
                np.reshape(self.pre_thrust/self.load.M*self.mea_R[:,2], (3,1))
        zb_dot = -self.KP*ev - self.KV*(r_acc-acc) - self.load.M*r_jer
        zb_des_dot = zb_dot / np.linalg.norm(zb) - zb_des*((zb_des.T*zb_dot)/np.linalg.norm(zb))
        om_skew_des = R_des.T * zb_des_dot
        om_x_des = -om_skew_des[1,0]
        om_y_des = om_skew_des[0,0]
        om_z_des = 0. # our choice
        om_des = np.matrix([
            [om_x_des],
            [om_y_des],
            [om_z_des]
        ])
        yaw_dot = -(om_y_des*(yc_des.T*yb_des))*(yc_des.T*zb_des)/zb_des[2,0]
        self.des_yaw += yaw_dot[0,0] * self.load.dT # update desired yaw online

        temp = (R_des.T * self.mea_R - self.mea_R.T * R_des)
        eR = 0.5 * np.matrix( [ [temp[2,1]], [temp[0,2]], [temp[1,0]] ] )
        ew = self.mea_wb - self.mea_R.T*R_des*om_des

        tau = -self.KR * eR - self.KW * ew + \
                np.cross(self.mea_wb, self.load.J*self.mea_wb, axisa=0, axisb=0).reshape((3,1)) - \
                self.load.J*(np.cross(self.mea_wb, self.mea_R.T*R_des*om_des, axisa=0, axisb=0).reshape((3,1)))
        self.pre_thrust = thrust_des

        # logging
        if log and self.id==0:
            self.des_pos_hist.append( np.copy(r_pos) )
            self.ref_fz_hist.append( np.copy(r_fz) )
            self.ref_tau_hist.append( np.copy(r_tau) )
            self.fz_hist.append( np.copy(-thrust_des) )
            self.tau_hist.append( np.copy(tau) )

        return -thrust_des, tau

    def local_wrench_basic(self, r_euler, r_wb, r_pos, r_vel, r_acc, r_R, use_cvxgen=True):
        """
        basic distributed wrench control without compensation
        """
        tol_fz, tol_tau = self.se3_ctrl(r_euler, r_wb, r_pos, r_vel, r_acc, r_R)
        fz = tol_fz / self.tol_bot
        tau = tol_tau / self.tol_bot

        # decentralized control using convex optimization
        assert(fz <= 0)
        tau_s = (self.W_o2s * np.vstack((fz,tau)))[1:4, :]
        ff = Variable(4)
        if not use_cvxgen:
            wrench = self.W_m2s * ff
            objective = Minimize( norm(wrench[2,0] - tau_s[1,0], 1) )
            constraints = [
                ff <= 0.0,
                ff >= -self.FMAX_PER_MOTOR,
                wrench[0,0] == fz,
                wrench[1,0] == tau_s[0,0],
                wrench[3,0] == tau_s[2,0],
            ]
            prob = Problem(objective, constraints)
            result = prob.solve(solver=CVXOPT)
        else: # use cvxgen
            ff.value = cvxgen_se3_solve(self.W_m2s, fz, tau_s, -self.FMAX_PER_MOTOR, 0.0)

        try:
            new_wrench = self.W_m2o * ff.value
        except:
            print prob.status, fz, tau, ff.value

        fz = new_wrench[0,0]
        tau = new_wrench[1:4, 0]
        
        # print ff.value
        
        # logging
        if self.id == 0:
            self.des_pos_hist.append( np.copy(r_pos) )

        return fz, tau

    def local_wrench(self, r_euler, r_wb, r_pos, r_vel, r_acc, r_R, r_jer=None):
        """
        Distributed local wrench controller, as described in the ICRA 18 paper
        """
        fz, tau = self.se3_ctrl_online_yaw(
            r_euler, r_wb, r_pos, r_vel, r_acc, r_R, r_jer=r_jer, log=False, noise_sensor=False)
        fz /= self.tol_bot
        tau /= self.tol_bot
        assert(fz <= 0)

        # if self.id == 0:
        #     print "SE(3) result", fz*self.tol_bot, tau[0]*self.tol_bot, tau[1]*self.tol_bot, tau[2]*self.tol_bot
        
        tau_s = (self.W_o2s * np.vstack((fz,tau)))[1:4, :] # desired tau in the special frame

        ### find pstar, described in the paper, using cvxgen
        BIAS = 100.0
        tau_temp = np.copy(tau_s) # 3x1 matrix, second element is tau_y
        tau_temp[1,0] = BIAS # disable the 1-norm in the cvxgen_se3_solve obj. 
                             # Here we give a big positive bias to reuse the cvxgen code to solve pmin and pmax
        fopt_temp = cvxgen_se3_solve(self.W_m2s, fz, tau_temp, -self.FMAX_PER_MOTOR, 0.0)
        result = self.W_m2s * fopt_temp
        pmin = result[2,0] + self.D*fz
        
        fopt_temp = cvxgen_se3_solve(self.W_temp, fz, tau_temp, -self.FMAX_PER_MOTOR, 0.0)
        result = self.W_m2s * fopt_temp
        pmax = result[2,0] + self.D*fz
        
        pstar = min([ math.fabs(pmin), math.fabs(pmax) ])
        
        ### torque compensation after considering the asymmetric deadband
        if tau_s[1,0] > 0.0:  # fz negative here
            tau_comp = 2.0 * tau_s[1,0] - self.D*fz - pstar # fz negative here
        else:
            tau_comp = tau_s[1,0]

        tau_s[1,0] = tau_comp
        f_comp = cvxgen_se3_comp_solve(
                self.W_m2s, fz, tau_s, -self.FMAX_PER_MOTOR, 0.0, -self.D*fz-pstar, -self.D*fz + pstar)
        new_wrench = self.W_m2o * f_comp
        ### TODO: Error handling

        fz = new_wrench[0,0]
        tau = new_wrench[1:4, 0]
        
        # logging
        self.des_pos_hist.append( np.copy(r_pos) )

        return fz, tau

    def get_eR(self, r_euler, r_wb, r_pos, r_vel, r_acc, r_R):
        fz = 0
        tau = np.matrix('0.0; 0.0; 0.0')

        # self.sense_groundtruth()
        self.sense()

        ep = self.mea_pos - r_pos
        ev = self.mea_vel - r_vel
        zw = np.matrix('0.0; 0.0; 1.0')
        Fdes = (-self.KP * ep - self.KV * ev + - self.load.M * self.load.g * zw + self.load.M * r_acc) / self.tol_bot
        
        zb = self.mea_R[:,2]
        fz = np.dot(Fdes.T, zb)[0,0]

        zb_des = - Fdes / np.linalg.norm(Fdes)
        xc_des = np.matrix([
            [np.cos(r_euler[2,0])],
            [np.sin(r_euler[2,0])],
            [0.0]
        ])
        zb_des_cross_xc_des = np.cross(zb_des, xc_des, axisa = 0, axisb = 0).reshape((3,1))
        mag_zb_des_cross_xc_des = np.linalg.norm(zb_des_cross_xc_des)
        yb_des = zb_des_cross_xc_des / mag_zb_des_cross_xc_des
        xb_des = np.cross(yb_des, zb_des, axisa = 0, axisb = 0).reshape((3,1))
        R_des = np.hstack((xb_des, yb_des, zb_des))

        temp = (R_des.T * self.mea_R - self.mea_R.T * R_des)
        eR = 0.5 * np.matrix( [ [temp[2,1]], [temp[0,2]], [temp[1,0]] ] )

        return eR