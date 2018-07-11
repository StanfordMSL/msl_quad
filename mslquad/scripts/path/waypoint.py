# Description: Easy interface to define waypoints
# Author: Zijian Wang, Stanford University
# Date: Jul 25, 2017
# Email: zjwang@stanford.edu

import numpy as np

class Keyframes:
    """
    class for managing keyframes more conveniently
    """
    def __init__(self, init_x, init_y, init_z, init_psi):
        """
        assumed the initial vel and acc are zeros
        """
        self.wps = None
        self.ts = [] # duration of each section

        init_wp = np.array([
            [init_x, 0.0, 0.0],
            [init_y, 0.0, 0.0],
            [init_z, 0.0, 0.0],
            [init_psi, 0.0, 0.0]
        ])

        self.wps = init_wp.reshape((1, 4, 3))

    def add_waypoint_pos_only(self, T, x, y, z, psi = 0.0):
        new_wp = np.array([
            [x, 0.0, 0.0],
            [y, 0.0, 0.0],
            [z, 0.0, 0.0],
            [psi, 0.0, 0.0]
        ])
        self.wps = np.concatenate((self.wps, new_wp.reshape(1,4,3)), axis = 0)
        self.ts.append(T)

    def add_waypoint_full(self, T, wp):
        """
        specify the waypoint with position, velocity and acceleration info
        inputs:
            T: duration fo the section
            wp: numpy array, 4 x 3, 1st col position, 2nd col velocity, 3rd col acceleration
        """
        self.wps = np.concatenate((self.wps, wp.reshape(1,4,3)), axis = 0)
        self.ts.append(T)   



class KeyframesPool:
    """
    store a bunch of keyframes for testing
    """
    def __init__(self):
        pass     

    def get_keyframes(self, name):
        """
        retrieve pre-defined keyframes
        inputs:
            name: name of the keyframes
        outputs:
            kf: Keyframes() object
        """
        if name == "000":
            kf = Keyframes(init_x = 0.0, init_y = 0.0, init_z = 0.0, init_psi = 0.0)
            t1 = 3.0
            wp2 = np.array([
                [ 1.0, -0.2, 0.0 ], # x, x_vel, x_acc
                [ 0.8, -0.2, 0.0 ], # y
                [ -0.5, -0.1, 0.0 ], # z
                [ 0.0, 0.0, 0.0 ], # psi
            ])
            t2 = 4.0
            wp3 = np.array([
                [ -0.5, 0.1, 0.0 ], # x, x_vel, x_acc
                [ -0.8, 0.2, 0.0 ], # y
                [ -1.0, 0.3, 0.0 ], # z
                [ 0.0, 0.0, 0.0 ], # psi
            ])
            t3 = 3.0
            wp4 = np.array([
                [ 0.0, 0.0, 0.0 ], # x, x_vel, x_acc
                [ 0.0, 0.0, 0.0 ], # y
                [ 0.0, 0.0, 0.0 ], # z
                [ 0.0, 0.0, 0.0 ], # psi
            ])
            kf.add_waypoint_full(t1, wp2)
            kf.add_waypoint_full(t2, wp3)
            kf.add_waypoint_full(t3, wp4)

        elif name == '001':
            kf = Keyframes(init_x = 0.0, init_y = 0.0, init_z = 0.0, init_psi = 0.0)
            kf.add_waypoint_pos_only(T = 1.5, x = 0.0, y = 0.0, z = -0.5)
            kf.add_waypoint_pos_only(T = 1.6, x = 1.5, y = 2.0, z = -1.5)
            kf.add_waypoint_pos_only(T = 1.2, x = 1.5, y = 2.0, z = -1.5)
            kf.add_waypoint_pos_only(T = 2.0, x = 0.0, y = 0.0, z = -0.1)
            kf.add_waypoint_pos_only(T = 0.5, x = 0.0, y = 0.0, z = 0.0)

        elif name == '002':
            kf = Keyframes(init_x = 0.0, init_y = 0.0, init_z = 0.0, init_psi = 0.0)
            kf.add_waypoint_pos_only(T = 1.3, x = 0.0, y = 0.0, z = -0.5)
            kf.add_waypoint_pos_only(T = 1.3, x = 0.5, y = 1.0, z = -1.0)
            kf.add_waypoint_pos_only(T = 1.5, x = 1.5, y = 2.0, z = -1.5)
            kf.add_waypoint_pos_only(T = 1.8, x = 0.0, y = 2.5, z = -0.9)
            kf.add_waypoint_pos_only(T = 1.8, x = 1.5, y = 0.0, z = -1.5)
            kf.add_waypoint_pos_only(T = 3.5, x = 0.0, y = 0.0, z = -0.4)
            kf.add_waypoint_pos_only(T = 1.2, x = 0.0, y = 0.0, z = 0.0)

        elif name == '003': # Waypoints used in ICRA paper
            kf = Keyframes(init_x = 0.0, init_y = 0.0, init_z = 0.0, init_psi = 0.0)
            kf.add_waypoint_pos_only(T = 2.6, x = 1.0, y = 0.8, z = -1.0)
            kf.add_waypoint_pos_only(T = 2.6, x = 2.0, y = 1.5, z = -2.0)
            kf.add_waypoint_pos_only(T = 2.6, x = 1.0, y = 3.0, z = -1.0)
            kf.add_waypoint_pos_only(T = 3.5, x = -1.0, y = 5.0, z = -0.5)
            kf.add_waypoint_pos_only(T = 4.2, x = 0.0, y = 7.0, z = -1.0)

        else:
            print('[Error] Found no existing keyframes named ' + name)
            return None
        
        return kf


if __name__ == "__main__":
    kfpool = KeyframesPool()
    kf = kfpool.get_keyframes(name = '000')
    print kf.wps