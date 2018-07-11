# Description: Implemented the abstraction of object dynamics in 3D
# Author: Zijian Wang, Stanford University
# Date: Jul 18, 2017
# Email: zjwang@stanford.edu

import numpy as np

def rot_mat(euler):
    """
    rotation matrix from body from to world frame
    euler is a 3x1 numpy matrix
    """
    assert( euler.shape == (3,1) )
    r = euler[0,0] # phi, roll angle
    p = euler[1,0] # theta, pitch angle
    y = euler[2,0] # psi, yaw angle

    return np.matrix([[ np.cos(p)*np.cos(y), np.sin(r)*np.sin(p)*np.cos(y)-np.cos(r)*np.sin(y), np.cos(r)*np.sin(p)*np.cos(y)+np.sin(r)*np.sin(y)],
      [np.cos(p)*np.sin(y), np.sin(r)*np.sin(p)*np.sin(y)+np.cos(r)*np.cos(y), np.cos(r)*np.sin(p)*np.sin(y)-np.sin(r)*np.cos(y)],
      [-np.sin(p), np.sin(r)*np.cos(p), np.cos(r)*np.cos(p)]])

def wb_2_thete_dot(wb, euler):
    """
    body-frame angular rate the euler angle rate, using quadrotor kinematics
    """
    assert( euler.shape == (3,1) )
    assert( wb.shape == (3,1) )
    r = euler[0,0] # phi, roll angle
    p = euler[1,0] # theta, pitch angle
    y = euler[2,0] # psi, yaw angle
    mat = np.matrix('0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0')
    mat[0,0] = 1
    mat[0,1] = np.sin(r)*np.tan(p)
    mat[0,2] = np.cos(r)*np.tan(p)
    mat[1,1] = np.cos(r)
    mat[1,2] = -np.sin(r)
    mat[2,1] = np.sin(r)/np.cosh(p) # np.sin(r)*sec(p)
    mat[2,2] = np.cos(r)/np.cosh(p) # np.cos(r)*sec(p)

    return mat * wb

def skew_sym(vec):
    """
    return skew symmetric matrix
    vec is a 3x1 numpy matrix
    """
    assert( vec.shape == (3,1) )
    v1 = vec[0,0]
    v2 = vec[1,0]
    v3 = vec[2,0]
    mat = np.matrix('0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0')
    mat[0,1] = -v3
    mat[0,2] = v2
    mat[1,0] = v3
    mat[1,2] = -v1
    mat[2,0] = -v2
    mat[2,1] = v1

    return mat
    
def quat_multiply(p, q):
    """
    p, q must be numpy matrix with dimension (4,1)
    """
    p = np.resize(p, (4,1))
    q = np.resize(q, (4,1))
    return np.matrix([
            [p[0,0]*q[0,0]-p[1,0]*q[1,0]-p[2,0]*q[2,0]-p[3,0]*q[3,0]],
            [p[0,0]*q[1,0]+p[1,0]*q[0,0]+p[2,0]*q[3,0]-p[3,0]*q[2,0]],
            [p[0,0]*q[2,0]-p[1,0]*q[3,0]+p[2,0]*q[0,0]+p[3,0]*q[1,0]],
            [p[0,0]*q[3,0]+p[1,0]*q[2,0]-p[2,0]*q[1,0]+p[3,0]*q[0,0]]
        ])

def quat_conj(q):
    q = np.resize(q, (4,1))
    return np.matrix([
            [q[0,0]],
            [-q[1,0]],
            [-q[2,0]],
            [-q[3,0]]
        ])

def quat_to_rot_mat(q):
    """
    convert quaternion to rotation matrix (ZYX sequence)
    """
    q1 = q[0,0]
    q2 = q[1,0]
    q3 = q[2,0]
    q4 = q[3,0]
    q1s = q1*q1
    q2s = q2*q2
    q3s = q3*q3
    q4s = q4*q4
    return np.matrix([
            [q1s+q2s-q3s-q4s, 2.0*(q2*q3+q1*q4) ,2.0*(q2*q4-q1*q3)],
            [2.0*(q2*q3-q1*q4), q1s-q2s+q3s-q4s, 2.0*(q3*q4+q1*q2)],
            [2.0*(q2*q4-q1*q3), 2.0*(q3*q4-q1*q2), q1s-q2s-q3s+q4s]
        ])

def quat_to_euler(q):
    """
    Assumed ZYX sequence
    """
    q0 = q[0,0]
    q1 = q[1,0]
    q2 = q[2,0]
    q3 = q[3,0]
    return np.matrix([
            [np.arctan(2.0*(q0*q1+q2*q3)/(1.0-2.0*(q1**2+q2**2)))],
            [np.arcsin(2.0*(q0*q2-q3*q1))],
            [np.arctan(2.0*(q0*q3+q1*q2)/(1.0-2.0*(q2**2+q3**2)))]
        ])

class LoadSim:
    """
    Object dyanmics. Assumed Z-down
    """
    def __init__(self, mass=2.0, J=None, radius=0.2, dT=0.01, use_quat=True):
        """
        mass: mass of the load, unit: kg
        J: moment of inertia, must be a 3x3 numpy.matrix
        radius: only for visualization
        dT: simulation time step, unit: second
        """
        self.use_quat = use_quat # use quaternion or euler angle for dynamics simulation
        # simulation params
        self.dT = dT # time step
        self.g = 9.8 # z down, so g is positive

        # geometries
        self.radius = radius

        # state
        self.euler = np.matrix([[0.0], [0.0], [0.0]])
        self.quat = np.matrix([[1.0], [0.0], [0.0], [0.0]])
        self.wb = np.matrix([[0.0], [0.0], [0.0]])
        self.pos = np.matrix([[0.0], [0.0], [0.0]])
        self.vel = np.matrix([[0.0], [0.0], [0.0]])
        
        # mass properties
        self.M = mass
        if J is None:
            self.J = np.matrix([
                [self.M/12.0, 0.0, 0.0],
                [0.0, self.M/12.0, 0.0],
                [0.0, 0.0, self.M/6.0]
            ])
        else:
            self.J = J
        self.Jinv = np.linalg.inv(self.J)

        self.reset_traj_history()

    def set_state(self, euler=None, wb=None, pos=None, vel=None):
        """
        set the initial state of the load
        inputs: 3x1 numpy matrix
        """
        if euler is not None:
            assert(euler.shape == (3,1))
            self.euler = np.copy(euler)
        if wb is not None:
            assert(wb.shape == (3,1))
            self.wb = np.copy(wb)
        if pos is not None:
            assert(pos.shape == (3,1))
            self.pos = np.copy(pos)
        if vel is not None:
            assert(vel.shape == (3,1))
            self.vel = np.copy(vel)

    def get_state(self):
        """
        get the state of the load
        must copy to avoid messing up with the matrix object reference
        return: euler, wb, pos, vel
        """
        return np.copy(self.euler), np.copy(self.wb), np.copy(self.pos), np.copy(self.vel)

    def get_rot_mat(self, euler = None):
        """
        get rotation matrix according to current euler angles
        """
        if euler is not None:
            return rot_mat(euler)
        else:
            if self.use_quat:
                return quat_to_rot_mat(self.quat)
            else:
                return rot_mat(self.euler)

    def reset_traj_history(self):
        self.euler_hist = []
        self.wb_hist = []
        self.pos_hist = []
        self.vel_hist = []
        self.pos_err_hist = [] 
        self.eR_hist = []

    def xdot(self, fz, tau):
        '''
        calculate xdot using dynamics equation
        '''
        assert(tau.shape == (3,1))

        wb_dot = self.Jinv * (tau - skew_sym(self.wb) * self.J * self.wb)
        pos_dot = self.vel

        e3 = np.matrix([[0.0], [0.0], [1.0]])
        if self.use_quat:
            quat_dot = 0.5*quat_multiply(self.quat, np.matrix([[0.0],[self.wb[0,0]],[self.wb[1,0]],[self.wb[2,0]]]))
            fz_q = np.matrix([[0.0], [0.0], [0.0], [fz]])
            temp = quat_multiply(quat_multiply(self.quat, fz_q), quat_conj(self.quat))
            temp2 = np.matrix([ [temp[1,0]], [temp[2,0]], [temp[3,0]] ])
            vel_dot = self.g * e3 + 1.0 / self.M * temp2
            return quat_dot, wb_dot, pos_dot, vel_dot
        else:
            euler_dot = wb_2_thete_dot(self.wb, self.euler)
            R = rot_mat(self.euler)
            vel_dot = self.g * e3 + 1.0 / self.M * R * fz * e3
            return euler_dot, wb_dot, pos_dot, vel_dot

    def step_using_tol_ft(self, fz, tau):
        """
        run one time step, update state using the total external force/torque input
        """
        assert( fz < 1e-3 ) # z-down coordinate frame, only generate positive thrust
        if self.use_quat:
            quat_dot, wb_dot, pos_dot, vel_dot = self.xdot(fz, tau)
            self.quat += quat_dot * self.dT
            self.quat /= np.linalg.norm(self.quat) # re-normalize to 1 magnitude
            self.euler = quat_to_euler(self.quat)
        else:
            euler_dot, wb_dot, pos_dot, vel_dot = self.xdot(fz, tau)
            self.euler += euler_dot * self.dT
        self.wb += wb_dot * self.dT
        self.pos += pos_dot * self.dT
        self.vel += vel_dot * self.dT

        # logging
        self.euler_hist.append(np.copy(self.euler))
        self.wb_hist.append(np.copy(self.wb))
        self.pos_hist.append(np.copy(self.pos))
        self.vel_hist.append(np.copy(self.vel))

    def record_log_signal(self, ref_pos = None, eR = None):
        if ref_pos is not None:
            self.pos_err_hist.append(np.copy( np.linalg.norm(ref_pos-self.pos) ))
        
        if eR is not None:
            self.eR_hist.append(np.copy( np.linalg.norm(eR) ))