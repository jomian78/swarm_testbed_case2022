import rospy

import numpy as np
import numpy.random as npr
from utils import convert_phi2phik

from geometry_msgs.msg import Pose
from std_msgs.msg import Empty, UInt8, Float32MultiArray, String
from ergodic_humanswarmcollab_sim.msg import tablet

import os

class TargetDist(object):
    """
    Defines the target distribution that the quadrotors will explore.
    The distribution is initialized to uniform. Change self.tablet to True
    when using the tablet interface and uncomment the tablet comms message
    as well as the input array subscriber found in init.
    """
    def __init__(self, name, team, basis, num_pts=50.):

        self.agent_name = name
        self.agent_team = team
        
        self.basis = basis
        self.num_pts = num_pts
        
        grid = np.meshgrid(*[np.linspace(0, 1, num_pts) for _ in range(2)])
        #grid = np.meshgrid(*[np.arange(0, dl, 0.1) for dl in self.basis.dl])  # allows alternate shapes than 1x1

        print('grid shape: ')
        print(np.shape(grid))
        #self.num_pts = grid[0].shape
        
        self.grid = np.c_[grid[0].ravel(), grid[1].ravel()] # dim = (num_pts**2,2), n=2

        # Initialize with Uniform Exploration
        self.grid_vals = self.init_uniform_grid(self.grid)
        self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
        self.has_update = False

        # Store the opposing team's last received target_distribution (initialized to be uniform)
        self.other_team_grid_vals = self.grid_vals.copy()
        self.other_team_phik = self._phik.copy()

        rospy.Subscriber('/clear_locs', Empty, self.reset_callback)
        
        self._tanvas = False
        #rospy.Subscriber('/input', Float32MultiArray, self.array_callback)
        rospy.Subscriber('/tablet_comm', tablet, self.tablet_callback)

        rospy.Subscriber('/agent_team_status', String, self.team_callback)


    @property
    def phik(self):
        return self._phik

    @phik.setter
    def phik(self, phik):
        assert len(phik) == self.basis.tot_num_basis, 'phik not right dim'
        self._phik = self.phik.copy()

    def team_callback(self, msg):
        """ Receives messages denoting the team status of each agent. 
        If the agent changes teams, assign it the target distribution
        of the team it is now on. 
        """
        msg_str = msg.data
        msg_str_list = list(msg_str.split(" "))
        if (msg_str_list[0] == self.agent_name and self.agent_team != msg_str_list[1]):
            self.agent_team = msg_str_list[1]

            # swap the grid_vals for each team
            temp = self.grid_vals
            self.grid_vals = self.other_team_grid_vals
            self.other_team_grid_vals = temp

            self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
            self.other_team_phik = convert_phi2phik(self.basis, self.other_team_grid_vals, self.grid)
            self.has_update = True  #update replay buffer, and everything in ergodic_control.py

    def get_grid_spec(self):
        xy = []
        for g in self.grid.T:
            xy.append(
                np.reshape(g, newshape=(self.num_pts[0], self.num_pts[1]))
            )
        return xy, self.grid_vals.reshape(self.num_pts[0], self.num_pts[1])

    def init_uniform_grid(self, x):
        assert len(x.shape) > 1, 'Input needs to be a of size N x n'
        assert x.shape[1] == 2, 'Does not have right exploration dim'

        val = np.ones(x.shape[0])
        val /= np.sum(val)
        return val

    def reset_callback(self,data):
        """ Reset the agent's current target distribution to a uniform distribution. 
        """
        self.grid_vals = self.init_uniform_grid(self.grid)
        self._phik = convert_phi2phik(self.basis, self.grid_vals, self.grid)
        self.has_update = False

    # def array_callback(self,data):
    #     print(self.grid_vals.shape)
    #     val = data.data
    #     self.grid_vals = np.array(val)
    #     print(self.grid_vals.shape)
    #     self._phik = convert_phi2phik(self.basis,self.grid_vals,self.grid)
    #     print('updating distribution with array')
    #     self.has_update = True

    def tablet_callback(self,data):
        """ Receives target distributions from the touchscreens
        of both players; Updates the agent's current target
        distribution if the team is correct; Stores the
        target distribution from the other team. 
        """
        if (data.team == self.agent_team):
            val = np.array(data.data,dtype=np.float32)
            #print("val.shape: , self.grid_vals.shape: ")
            #print(val.shape,self.grid_vals.shape)
            self.grid_vals = val
            self._phik = convert_phi2phik(self.basis,self.grid_vals,self.grid)
            #print('updating the target distribution for team {}'.format(self.agent_team))
            self.has_update = True
        else:
            #rospy.logwarn("target received was for the other team")
            val = np.array(data.data,dtype=np.float32)
            self.other_team_grid_vals = val
            self.other_team_phik = convert_phi2phik(self.basis,self.other_team_grid_vals,self.grid)
