# Test script for C-extension of diff code
# Author: Zijian Wang, zjwang@stanford.edu
# Date: Jul 13, 2017
# Multi-Robot Systems Lab, Stanford University

import diff

''' How to use:
euler, wb, pos, vel, fz, tau, R = diff.diff2SttIn( m, J, Jinv, sig, sigd1, sigd2, sigd3, sigd4 )
Inputs:
    m: mass, double
    J: inertial matrix, 6x1 tuple containing: (Ixx, Iyy, Izz, Ixy, Iyz, Ixz)
    Jinv: J inverse. 6x1 tuple as above
    sig: flat output (trajectory), 4x1 tuple containing: (x(t), y(t), z(t), yaw(t))
    sigd1-d4: n-th order derivative of sig, 4x1 tuple
'''

m = 2.0
J = (1.0/6.0, 1.0/6.0, 1.0/3.0, 0.0, 0.0, 0.0)
Jinv = (6.0, 6.0, 3.0, 0.0, 0.0, 0.0)

sig = (1.0000, 0.0080, -0.5000, 0.0)
sigd1 = (-0.0064, 0.8000, 0.0, 0.0)
sigd2 = (-0.6400, -0.0051, 0.0, 0.0)
sigd3 = (0.0041, -0.5120, 0.0, 0.0)
sigd4 = (0.4096, 0.0033, 0.0, 0)

euler, wb, pos, vel, fz, tau, R = diff.diff2SttIn( m, J, Jinv, sig, sigd1, sigd2, sigd3, sigd4 )
print "euler:"
print euler
print "wb:"
print wb
print "pos:"
print pos
print "vel:"
print vel
print "fz:"
print fz
print "tau:"
print tau
print "R:"
print R

'''
# correct output should be: (verified in Matlab)
euler:
(-0.0005193019120206024, 0.06521351816716534, 0.0)
wb:
(-0.052133843766002316, -0.00041659057769280434, -2.1633630297243403e-07)
pos:
(1.0, 0.008, -0.5)
vel:
(-0.0064, 0.8, 0.0)
fz:
-19.6417540978
tau:
(5.5765622761269486e-05, -0.0069364062422583115, -1.444366805984892e-05)
R:
(0.9978743520145146, -3.384150437006753e-05, 0.0651672958343769, 0.0, 0.9999998651627651, 0.0005193018886801909, -0.06516730462135606, -0.0005181980356666591, 0.9978742174638962)
'''