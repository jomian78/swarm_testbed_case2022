import numpy as np
from basis import Basis
from replay_buffer import ReplayBuffer
from copy import deepcopy
import rospy
from std_msgs.msg import String, Float32MultiArray, Empty

from grid_map_msgs.msg import GridMap
from ergodic_humanswarmcollab_sim.msg import Ck

from matplotlib import pyplot as plt ## plotting for debugging
from matplotlib.colors import LinearSegmentedColormap, Normalize

from utils import convert_ck2dist, convert_phik2phi

from itertools import compress

from collections import defaultdict ## used for creating a ck dict that has 3 values for each key value

"""
Plotting of spatial and temporal distributions for decentralized ergodic
controller by agent. Takes in an agent identifer,
number of basis functions to represent the trajectory in Fourier space, and
number of points per meter to use to specify the spatial distribution
"""

class DErgPlot(object):
    def __init__(self, agent_name, id_number, num_basis=10, num_pts=50.0, capacity=100000):

        self.got_explr_space    = False
        self._agent_name        = agent_name
        self._agent_id          = id_number
        if (self._agent_id == 0 or self._agent_id % 2 == 0):
            self._agent_team = "blue"
        else:
            self._agent_team = "red"
        rospy.Subscriber('/agent_team_status', String, self.team_callback)
            
        #self._ck_dict          = {}
        self._ck_dict           = defaultdict(list)       #format: agent_name -> agent_team, ck, time_ck_was_received
        self._replay_buffer     = ReplayBuffer(capacity)
        self.num_pts            = num_pts
        self.num_plots          = 4  # this can be 2 or 4

        rospy.init_node(agent_name+'_plot')
        rospy.Subscriber(agent_name+'/target_dist', GridMap, self._targ_dist_callback)
        rospy.Subscriber('ck_link', Ck, self._ck_link_callback)
        rospy.Subscriber('/clear_locs', Empty, self.reset_callback)
        self._rate = rospy.Rate(1)

        while self.got_explr_space is False:
            self._rate.sleep()

        self._basis = Basis(self.explr_space, num_basis=num_basis)

        show_lamk = False
        if show_lamk:
            plt.figure(figsize=[4,3])
            plt.title('$\Lambda_k$')
            lamk1 =  np.exp(-0.8*np.linalg.norm(self._basis.k, axis=1))
            lamk2 =  (1 + np.square(np.linalg.norm(self._basis.k, axis=1)))**(-3/2)
            plt.plot(lamk1, label='$exp(-0.8 ||k||)$')
            plt.plot(lamk2, label='$(1+||k||^2)^{-(v+1)/2}$')
            plt.legend()
            plt.show()

        if self.num_plots == 2:
            fig, (self.ax0, self.ax1) = plt.subplots(ncols=2,figsize=[3,3])
            axes = (self.ax0, self.ax1)
        else:
            fig, (self.ax0, self.ax1, self.ax2, self.ax3) = plt.subplots(ncols=4,figsize=[6,3])
            axes = (self.ax0, self.ax1, self.ax2, self.ax3)
        # Set the ticks and ticklabels for all axes
        x_pos, y_pos = [np.linspace(0,self.num_pts[idx]-1,3) for idx in [1,0]]
        x_name, y_name = [np.linspace(0,self.explr_space[1,idx],3) for idx in range(2)]
        # plt.setp(axes, xticks=x_pos, xticklabels=x_name, yticks=y_pos, yticklabels=y_name) # got messed up when I changed the number of pts
        
        plt.ion()
        if self.num_plots == 2:
            self.ax0.set_title('spatial dist') # phi_k
            self.ax1.set_title('temporal dist') # ck
        else:
            self.ax0.set_title('spatial dist') # phi_k
            self.ax1.set_title('temporal dist\n (current agent)') # ck
            self.ax2.set_title('temporal dist\n (avg all agents)') # ck_mean agents
            self.ax3.set_title('temporal dist\n (avg current agent)') # ck_mean over time
        plt.suptitle(self._agent_name)
        plt.show()

        # custom color map for our game environment
        colors = [(0,1,0), (0,0,1)] #Green -> Blue
        n_bins = 100
        self.cmap_name = 'green_blue';
        self.colormap = LinearSegmentedColormap.from_list(self.cmap_name, colors, N=n_bins)

        # custom colorbar for our game environment
        self.sm = plt.cm.ScalarMappable(Normalize(0,1), cmap=self.colormap)
        self.sm.set_array(range(0,1))
        if self.num_plots == 2:
            self.cax = fig.add_axes([self.ax1.get_position().x1 + 0.02, self.ax1.get_position().y0 + 0.25, 0.02, self.ax1.get_position().height - 0.5])
            self.cbar_ax = fig.colorbar(self.sm, cax=self.cax, ticks=[0,1])
            self.cbar_ax.set_ticklabels(['Low', 'High'])
        else:
            self.cax = fig.add_axes([self.ax3.get_position().x1 + 0.02, self.ax3.get_position().y0 + 0.25, 0.02, self.ax3.get_position().height - 0.5])
            self.cbar_ax = fig.colorbar(self.sm, cax=self.cax, ticks=[0,1])
            self.cbar_ax.set_ticklabels(['Low', 'High'])

    def team_callback(self, msg):
        ''' receive the team this agent is on. '''
        msg_str = msg.data
        msg_str_list = list(msg_str.split(" "))
        if self._agent_name == msg_str_list[0]:
            if (self._agent_team == msg_str_list[1]):
                pass # no change needed
            else:
                self._agent_team = msg_str_list[1]
                self.reset() #reset the replay buffer
        else:
            pass

    def _targ_dist_callback(self, msg):
        if self.got_explr_space is False:
            self.explr_space = np.array([[0.,0.],[msg.info.length_x/10.0, msg.info.length_y/10.0]], dtype=np.float32)
            
            grid = np.meshgrid(*[np.arange(0, self.explr_space[1,idx], 1/self.num_pts) for idx in range(2)])  # allows alternate shapes than 1x1
            
            self.num_pts = grid[0].shape
            
            self.grid = np.c_[grid[0].ravel(), grid[1].ravel()] # dim = (num_pts**2,2), n=2
            
            self.got_explr_space = True
        self.phi = np.flip(msg.data[0].data, axis=0)

    def _ck_link_callback(self, msg):
        msg_timestamp = rospy.get_time()
        if self._ck_dict.has_key(msg.name):
            self._ck_dict[msg.name][0] = msg.team
            self._ck_dict[msg.name][1] = np.array(msg.ck)
            self._ck_dict[msg.name][2] = msg_timestamp
        else:
            self._ck_dict.update({msg.name : [msg.team, np.array(msg.ck), msg_timestamp]})
        if msg.name == self._agent_name:
            self._replay_buffer.push(msg.ck)

    def reset(self):
        self._replay_buffer.reset()

    def step(self):
        if self._agent_name in self._ck_dict.keys():
            #print("ck_dict for {}".format(self._agent_name))
            #print(self._ck_dict[self._agent_name])
            
            ck = self._ck_dict[self._agent_name][1]

            # phi
            phi_plot = np.reshape(self.phi,(self.num_pts[0],self.num_pts[1]))
            self.ax0.imshow(phi_plot,origin='lower',interpolation='none', cmap=self.colormap)

            # ck
            ck_dist = convert_ck2dist(self._basis, ck.copy(),grid=self.grid)
            ck_plot = np.reshape(ck_dist,(self.num_pts[0],self.num_pts[1]))
            self.ax1.imshow(ck_plot,origin='lower',interpolation='none', cmap=self.colormap)

            if self.num_plots != 2:
                # ck_mean
                cks = []
                for key in self._ck_dict.keys():
                    if (self._ck_dict[key][0] == self._agent_team):
                        cks.append(self._ck_dict[key][1])
                ck = np.mean(cks, axis=0)

                ck_dist = convert_ck2dist(self._basis, ck.copy(),grid=self.grid)
                ck_plot = np.reshape(ck_dist,(self.num_pts[0],self.num_pts[1]))
                self.ax2.imshow(ck_plot,origin='lower',interpolation='none', cmap=self.colormap)

                # ck_mean (single agent)
                ck = np.mean(self._replay_buffer.buffer, axis=0)

                ck_dist = convert_ck2dist(self._basis, ck.copy(),grid=self.grid)
                ck_plot = np.reshape(ck_dist,(self.num_pts[0],self.num_pts[1]))
                self.ax3.imshow(ck_plot,origin='lower',interpolation='none', cmap=self.colormap)

            plt.draw()
            plt.pause(0.00000000001)

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            self._rate.sleep()

    def reset_callback(self,data):
        self.reset()
