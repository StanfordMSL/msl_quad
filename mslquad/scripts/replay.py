#!/usr/bin/env python

'''
The MIT License (MIT)
Copyright (c) 2019 Kunal Shah
                kshah.kunal@gmail.com
'''
# std lib imports
import sys
import time
import numpy as np
import numpy.linalg as la
# plot
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# Standard ROS message
import rospy
from std_msgs.msg import Bool, Time
from geometry_msgs.msg import Pose, PoseStamped, Twist

from quadProxy import QuadProxy


class Replay(object):
    """docstring for Replay
    this class impliments a reviwer to plot the trajs of the quads"""

    def __init__(self):
        # self.manifest = ["rexquad3"]
        self.manifest = ["rexquad0", "rexquad3", "quad3"]
        print(self.manifest)

        # make proxy objects
        self.proxies = [QuadProxy(quad) for quad in self.manifest]
        # empty list for the positions
        self.trajHist = [[], [], []]

        self.fig = plt.figure(facecolor='w', edgecolor='k')
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.colors = ['b', 'g', 'r']
        self.ax.set_xlim([-6, 2])
        self.ax.set_ylim([-1.5, 1.5])
        self.ax.set_zlim([0, 2.5])
        plt.ion()
        plt.draw()

        # make plot timer
        # self.plotter = rospy.Timer(rospy.Duration(.05), self.drawCB)

    def drawCB(self):
        for i, (quad, color) in enumerate(zip(self.proxies, self.colors)):
            self.trajHist[i].append([quad.pose.position.x,
                                     quad.pose.position.y,
                                     quad.pose.position.z])
            self.ax.scatter(quad.pose.position.x,
                            quad.pose.position.y,
                            quad.pose.position.z,
                            color=color)
        plt.draw()

    def run(self):
        rate = rospy.Rate(50)  # 10 Hz
        while not rospy.is_shutdown():
            # usrCmd = raw_input("enter to stop a commmand: ")
            #  self.plotter.shutdown()
            self.drawCB()
            plt.draw()
            plt.pause(.0000001)
            rate.sleep()
        for quadname, traj in zip(self.manifest, self.trajHist):
            print("traj for: " + quadname)
            for pt in traj:
                print(pt)


if __name__ == '__main__':
    # make tower
    player = Replay()
    player.run()
