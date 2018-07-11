# Description: Plotting utilities using matplotlib
# Author: Zijian Wang, Stanford University
# Date: Jul 18, 2017
# Email: zjwang@stanford.edu

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
from matplotlib import animation
import matplotlib

USE_LATEX = False

# for latex label, require latex installation
'''
from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)
USE_LATEX = True
'''

class VizTool:
    """
    visualization tool
    """
    def __init__(self, load, quads):
        self.load = load
        self.quads = quads
        self.fig = None # used for interactive mode

    def plot_traj_history(self):
        N = len(self.load.euler_hist)
        dT = self.load.dT
        t = np.linspace(0.0, N*dT, N)

        plt.ioff() # turn off interactive mode
        self.fig = None
        # plot actual position
        plt.figure(1)
        pos_x = [pos[0,0] for pos in self.load.pos_hist]
        pos_y = [pos[1,0] for pos in self.load.pos_hist]        
        pos_z = [pos[2,0] for pos in self.load.pos_hist]
        plt.plot(t, pos_x, linewidth=4)
        plt.plot(t, pos_y, linewidth=4)
        plt.plot(t, pos_z, linewidth=4)
        # plot desired position
        if len(self.quads[0].des_pos_hist) == len(self.load.pos_hist):
            des_pos_x = [pos[0,0] for pos in self.quads[0].des_pos_hist]
            des_pos_y = [pos[1,0] for pos in self.quads[0].des_pos_hist]
            des_pos_z = [pos[2,0] for pos in self.quads[0].des_pos_hist]
            plt.plot(t, des_pos_x, '--', linewidth=5)
            plt.plot(t, des_pos_y, '--', linewidth=5)
            plt.plot(t, des_pos_z, '--', linewidth=5)
            plt.legend(['pos x', 'pos y', 'pos z', 'des pos x', 'des pos y', 'des pos z'],loc=0)
        else:
            plt.legend(['pos x', 'pos y', 'pos z'])
        plt.xlabel("Time / s", fontsize=20)
        plt.ylabel("Position / m", fontsize=20)

        # plot euler angle
        plt.figure(2)
        r = [euler[0,0] for euler in self.load.euler_hist]
        p = [euler[1,0] for euler in self.load.euler_hist]        
        y = [euler[2,0] for euler in self.load.euler_hist]
        plt.plot(t, r, linewidth=4)
        plt.plot(t, p, linewidth=4)
        plt.plot(t, y, linewidth=4)
        plt.legend(['roll', 'pitch', 'yaw'], loc=0)
        plt.xlabel("Time / s", fontsize=20)
        plt.ylabel("Angle / rad", fontsize=20)
        matplotlib.rcParams.update({'font.size': 20})
        plt.show()

    def plot_force_torque(self, traj = None):
        fz = [np.asscalar(x) for x in self.quads[0].fz_hist]
        tau_x = [tau[0,0] for tau in self.quads[0].tau_hist]
        tau_y = [tau[1,0] for tau in self.quads[0].tau_hist]
        tau_z = [tau[2,0] for tau in self.quads[0].tau_hist]

        t = np.linspace(self.load.dT, len(fz)*self.load.dT, len(fz))

        if traj is not None:
            ref_fz = []
            ref_tau_x = []
            ref_tau_y = []
            ref_tau_z = []
            for tt in t:
                fout = traj.get_flatout(tt)
                r_euler, r_wb, r_pos, r_vel, r_fz, r_tau, r_R = traj.get_ref_state_input(t, fout)
                ref_fz.append(r_fz)
                ref_tau_x.append(r_tau[0,0])
                ref_tau_y.append(r_tau[1,0])
                ref_tau_z.append(r_tau[2,0])
        
        plt.figure(1)
        plt.plot(t,fz)
        if traj is not None:
            plt.plot(t, ref_fz, '--')

        plt.figure(2)
        plt.plot(t,tau_x)
        plt.plot(t,tau_y)
        plt.plot(t,tau_z)
        if traj is not None:
            plt.plot(t, ref_tau_x, '--')
            plt.plot(t, ref_tau_y, '--')
            plt.plot(t, ref_tau_z, '--')

        plt.show()

    def plot_pos_error(self):
        N = len(self.load.euler_hist)
        dT = self.load.dT
        t = np.linspace(0.0, N*dT, N)

        plt.ioff()
        plt.figure(1)

        
        err_pos = [ep for ep in self.load.pos_err_hist]
        plt.plot(t, err_pos, linewidth=4)

        matplotlib.rcParams.update({'font.size': 22})

        if USE_LATEX:
            plt.xlabel(r"Time / s", fontsize=26) # use latex
            plt.ylabel(r"$\| \ e_p \ \|$ / m", fontsize=36)
        else:
            plt.xlabel("Time / s", fontsize=26)
            plt.ylabel("eP / m", fontsize=26)

        plt.show()

    def plot_eR(self):
        N = len(self.load.euler_hist)
        dT = self.load.dT
        t = np.linspace(0.0, N*dT, N)

        plt.ioff()
        plt.figure(1)

        
        eR = [eR for eR in self.load.eR_hist]
        plt.plot(t, eR)

        matplotlib.rcParams.update({'font.size': 22})

        if USE_LATEX:
            plt.xlabel(r"Time / s", fontsize=26)
            plt.ylabel(r"$\| \ e_R \ \|$", fontsize=36)
        else:
            plt.xlabel("Time / s", fontsize=26)
            plt.ylabel("eR", fontsize=26)

        plt.show()
        

    def animate_call(self, i, trash, lines):
        """
        utility function used by animation
        """
        # set position of the object
        lines[0].set_data(self.load.pos_hist[i][0,:], self.load.pos_hist[i][1,:])
        lines[0].set_3d_properties(self.load.pos_hist[i][2,:])

        R = self.load.get_rot_mat(self.load.euler_hist[i])

        # set position for the frame_x
        armx0 = R * np.matrix([
            [-1.0],
            [0.0],
            [0.0],
        ]) + self.load.pos_hist[i]
        armx1 = R * np.matrix([
            [1.0],
            [0.0],
            [0.0],
        ]) + self.load.pos_hist[i]
        lines[1].set_data( 
            [armx0[0,:], armx1[0,:]],
            [armx0[1,:], armx1[1,:]]
        )
        lines[1].set_3d_properties([armx0[2,:], armx1[2,:]])
        # set position for the frame_y
        army0 = R * np.matrix([
            [0.0],
            [-1.0],
            [0.0],
        ]) + self.load.pos_hist[i]
        army1 = R * np.matrix([
            [0.0],
            [1.0],
            [0.0],
        ]) + self.load.pos_hist[i]
        lines[2].set_data( 
            [army0[0,:], army1[0,:]],
            [army0[1,:], army1[1,:]]
        )
        lines[2].set_3d_properties([army0[2,:], army1[2,:]])

        # set position for quads
        for k in range(3, len(lines)):
            qd = k - 3
            pos_b = np.matrix([ [self.quads[qd].X], [self.quads[qd].Y], [0.0] ])
            pos_w = R * pos_b
            pos = np.copy(pos_w + self.load.pos_hist[i])
            lines[k].set_data(pos[0,:], pos[1,:])
            lines[k].set_3d_properties(pos[2,:])

        # print i
        return lines
    
    def animation(self):
        """
        use this to show animation after the simulation is done
        will retrieve history from the load.xxx_hist
        """
        if self.fig is None:
            self.fig = plt.figure()
        ax = self.fig.add_subplot(111, projection='3d') 
        ax.hold(True)

        # initial object plot
        obj_pos = np.copy(self.load.pos_hist[0])
        line_obj = [ax.plot(obj_pos[0,:], obj_pos[1,:], obj_pos[2,:], 'yo', markersize = 15.0)[0] ]
        line_framex = [ ax.plot(
            [obj_pos[0,:]-1.0, obj_pos[0,:]+1.0], 
            [obj_pos[1,:], obj_pos[1,:]], 
            [obj_pos[2,:], obj_pos[2,:]],
            'b-', linewidth = 2.0
        )[0] ]
        line_framey = [ ax.plot(
            [obj_pos[0,:], obj_pos[0,:]], 
            [obj_pos[1,:]-1.0, obj_pos[1,:]+1.0], 
            [obj_pos[2,:], obj_pos[2,:]],
            'r-', linewidth = 2.0
        )[0] ]       

        # initial thruster plot
        R = self.load.get_rot_mat()
        quad_pos = []
        for quad in self.quads:
            pos_b = np.matrix([ [quad.X], [quad.Y], [0.0] ])
            pos_w = R * pos_b
            quad_pos.append( np.copy(pos_w + self.load.pos) )

        lines_quad = [ax.plot( quad_pos[i][0,:], quad_pos[i][1,:], quad_pos[i][2,:], 
                'ro', markersize = 4.0 )[0] for i in range(len(quad_pos)) ]

        # combine lines for animation
        lines = line_obj + line_framex + line_framey + lines_quad
        

        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_zlim(-3, 1)
        ax.set_xlabel('x axis')
        ax.set_ylabel('y axis')
        ax.set_zlabel('z axis')
        plt.gca().invert_zaxis() # z-down frame
        plt.gca().invert_xaxis()

        ani = animation.FuncAnimation(self.fig, self.animate_call, frames=len(self.load.euler_hist), fargs=(None, lines), blit=True, repeat=False, interval=1)
        #ani.save('matplot003.gif', writer='imagemagick')
        plt.show()

    def ros_animation(self):
        """
        visualize the flight in rviz
        """
        import rospy
        import tf
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(1.0/self.load.dT)
        for i in range(len(self.load.pos_hist)):
            euler = self.load.euler_hist[i]
            q = tf.transformations.quaternion_from_euler(euler[0,0], -euler[1,0], -euler[2,0])
            br.sendTransform((self.load.pos_hist[i][0,0], -self.load.pos_hist[i][1,0], -self.load.pos_hist[i][2,0]),
                            (q[0], q[1], q[2], q[3]),
                            rospy.Time.now(),
                            "base_link",
                            "map")
            rate.sleep()
            if rospy.is_shutdown():
                return


class TrajPlotter:
    def __init__(self):
        # fig1 and ax1 are used to plot the trajectory in 3D
        self.fig1 = plt.figure(1)
        self.ax1 = self.fig1.add_subplot(111, projection='3d') 

        # fig2,3 and ax2,3 are used to plot the reference state and input by differential flatness
        self.fig2 = plt.figure(2)
        self.ax2 = self.fig2.add_subplot(111)
        self.fig3 = plt.figure(3)
        self.ax3 = self.fig3.add_subplot(111)

    def plot_traj(self, traj, style = None):
        posx = []
        posy = []
        posz = []
        velx = []
        vely = []
        velz = []
        jerx = []
        jery = []
        jerz = []
        snax = []
        snay = []
        snaz = []

        T = sum(traj.ts) # total time of the trajectory
        for t in np.arange(0.0, T, 0.01):
            fout = traj.get_flatout(t)
            posx.append(fout[0,0])
            posy.append(fout[1,0])
            posz.append(fout[2,0])
            velx.append(fout[0,1])
            vely.append(fout[1,1])
            velz.append(fout[2,1])
            jerx.append(fout[0,2])
            jery.append(fout[1,2])
            jerz.append(fout[2,2])
            snax.append(fout[0,3])
            snay.append(fout[1,3])
            snaz.append(fout[2,3])

        if style is not None:
            self.ax1.plot(posx, posy, posz, style)
        else:
            self.ax1.plot(posx, posy, posz)

    def plot_ref_stt_in(self, traj, style = None):
        T = sum(traj.ts)
        fz = []
        taux = []
        tauy = []
        tauz = []
        for t in np.arange(0.0, T, 0.01):
            fout = traj.get_flatout(t)
            r_euler, r_wb, r_pos, r_vel, r_fz, r_tau, r_R = traj.get_ref_state_input(t, fout)
            fz.append(-r_fz)
            taux.append(r_tau[0,0])
            tauy.append(r_tau[1,0])
            tauz.append(r_tau[2,0])

        t = np.arange(0.0, T, 0.01)

        if style is not None:
            self.ax2.plot(t, fz, style, linewidth=3)
            self.ax3.plot(t,taux,style, t,tauy,style, t,tauz,style, linewidth=2.5)

            # non general code, only for paper plotting, please comment out
            '''
            force_lb = np.array([19.8-0.5]*t.size)
            force_ub = np.array([19.8+0.5]*t.size)
            self.ax2.plot(t, force_lb, 'k:', t, force_ub, 'k:')
            moment_lb = np.array([-0.01]*t.size)
            moment_ub = np.array([0.01]*t.size)
            self.ax3.plot(t, moment_lb, 'k:', t, moment_ub, 'k:')
            '''

        else:
            self.ax2.plot(t, fz)
            self.ax3.plot(t, taux)
            self.ax3.plot(t, tauy)
            self.ax3.plot(t, tauz)            


    def show(self):    
            # change font size of all elements
        matplotlib.rcParams.update({'font.size': 16})

        self.ax1.set_xlim(-3, 3)
        self.ax1.set_ylim(-1, 6)
        self.ax1.set_zlim(-3, 1)
        self.ax1.set_xlabel('x axis')
        self.ax1.set_ylabel('y axis')
        self.ax1.set_zlabel('z axis')
        self.ax1.invert_zaxis() # z-down frame
        self.ax1.invert_xaxis()

        # self.ax2.set_title('Reference thrust fz')
        self.ax2.set_xlabel('t / s', fontsize=22)
        self.ax2.set_ylabel('thrust', fontsize=22)
        # self.ax3.set_title('Reference moment tau')
        self.ax3.set_xlabel('t / s', fontsize=22)
        self.ax3.set_ylabel('moments', fontsize=22)

        # non-general code, only for the paper plots. please comment out
        '''
        g1 = plt.Line2D((0,1),(0,0), linestyle='--', color='b', linewidth=3)
        g2 = plt.Line2D((0,1),(0,0), linestyle='-', color='g', linewidth=3)
        g3 = plt.Line2D((0,1),(0,0), linestyle=':', color='k')
        self.ax2.legend([g1, g2, g3],['Manual chosen duration', 'Optimized duration', 'Desired bound'], loc=4)

        h1 = plt.Line2D((0,1),(0,0), linestyle='--', linewidth=2.5)
        h2 = plt.Line2D((0,1),(0,0), color='c', linestyle='-', linewidth=2.5)
        h3 = plt.Line2D((0,1),(0,0), color='k', linestyle=':')
        self.ax3.legend([h1, h2, h3],['Manual chosen duration', 'Optimized duration', 'Desired bound'])
        '''

        plt.show()