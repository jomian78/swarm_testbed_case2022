#!/usr/bin/env python

# Based on paper: Hauert, S., Leven, S., Varga, M., Ruini, F., Cangelosi, A., Zufferey, J. C., & Floreano, D. (2011). "Reynolds flocking in reality with fixed-wing robots: communication range vs. maximum turning rate."" In Proceedings of the IEEE/RSJ International Conf. on Intelligent Robots and Systems.

import rospy
import numpy as np
from copy import deepcopy
from std_msgs.msg import String, Float32MultiArray, Empty
from matplotlib import pyplot as plt ## plotting for debugging

from ergodic_humanswarmcollab_sim.msg import FlockingPose, FlockingRelativeAngle, FlockingCmd

from collections import defaultdict ## used for creating a dict that has several values for each key value

def R(theta):
    return np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])

def getVec(to_loc,from_loc,theta):
    dxdy = to_loc-from_loc          # change in global x/y direction
    vec = R(theta).dot(dxdy)      # rotate vec to
    return vec

def getAngleAndDistance(vec):
    angle = np.arctan2(vec[1],vec[0])
    distance = np.linalg.norm(vec)
    return angle,distance

def wrap(angle):
	# angle wrapping from -pi to +pi
    if angle > np.pi:
    	angle -= 2*np.pi
    elif angle < -np.pi:
    	angle += 2*np.pi
    return angle

def normalize(vec, final=False):
    norm = np.linalg.norm(vec)
    if norm > 2.:
        vec /= norm
    elif final and (norm < 0.1):
        vec[0] += 0.1
    return vec


class DFlockingControl(object):
    """
    Decentralized flocking for each agent. Takes in an agent identifer and
    team identifier. The controller outputs a set of controls to be executed by the agent.

    """
    def __init__(self, agent_name, agent_team):

        self._agent_name    = agent_name
        self._agent_team    = agent_team
        rospy.Subscriber('/agent_team_status', String, self.team_callback)

    	# initialize empty vectors for implementations
    	self.migr_point = np.array([0.25,0.4])	    # x,y location of migration point
    	# initialize weights and thresholds (but our area is much smaller so had to tune these)
    	self._weight_align = 0.1  	 	# 1 from paper
    	self._weight_coh   = 0.1 		# 1 from paper
    	self._weight_sep   = 0.15 		# 1.2 from paper
    	self._weight_migr  = 100.  	  	# 1/500 from paper
        self._thresh_comm  = 0.15 	    # limit communication distance to those within threshold
    	self._thresh_sep   = 0.05 		# limit separation distance (repelling robots) to those within (closer) threshold
    	self._thresh_turn  = np.pi/10.   # turn when angle from combined vector exceeds threshold (radians)

    	# initialize empty dict to keep info on other robots in swarm
    	self._agent_dict   = defaultdict(dict)
        self.agents_to_remove = []

        rospy.Subscriber('agent_pose_link', FlockingPose, self._pose_link_callback)
        self._agent_pose_pub = rospy.Publisher('agent_pose_link',FlockingPose, queue_size=1)
        rospy.Subscriber('agent_angle_link', FlockingRelativeAngle, self._angle_link_callback)
        self._agent_angle_pub = rospy.Publisher('agent_angle_link', FlockingRelativeAngle, queue_size=1)
        rospy.Subscriber('flocking_command', FlockingCmd, self._cmd_callback)

    def team_callback(self, msg):
        ''' receive the team this agent is on. '''
        msg_str = msg.data
        msg_str_list = list(msg_str.split(" "))

        if (self._agent_name == msg_str_list[0] and self._agent_team != msg_str_list[1]):
                self._agent_team = msg_str_list[1]
            	self._agent_dict = defaultdict(dict)

    def _cmd_callback(self, msg):
        ''' update migration data '''
        self.migr_point[0] = msg.migration_x
        self.migr_point[1] = msg.migration_y
        self._weight_migr = msg.migration_weight

    def _pose_link_callback(self, msg):
		# check if robot id is in list and on team
        if (msg.name != self._agent_name) and (msg.team == self._agent_team):
            # add/update if on team
            if self._agent_dict.has_key(msg.name):
                self._agent_dict[msg.name]['rx_state'] = np.array(msg.pose)
            else:
                self._agent_dict.update({msg.name : {'rx_state': np.array(msg.pose), 'rx_angle': 0.}})
        else:
            if self._agent_dict.has_key(msg.name): # remove if not on team
                self.agents_to_remove.append(msg.name)

    def _angle_link_callback(self, msg):
		# check if robot id on team
        if (msg.sent_to_agent == self._agent_name):
            # update if on team
            if self._agent_dict.has_key(msg.sent_from_agent):
                self._agent_dict[msg.sent_from_agent]['rx_angle'] = msg.angle


    def reset(self):
        pass

    def __call__(self, state):  # state = [x, y, xdot, ydot]

        # get current location
        xy_pose = state[:2]
        vel = state[2:]
        theta,_ = getAngleAndDistance(vel)

        # send message about pose to other agents
        msg = FlockingPose(self._agent_name, self._agent_team, xy_pose)
        self._agent_pose_pub.publish(msg)

        ## compute migration vector in robot frame (weighted by distance)
        v_migr = getVec(self.migr_point, xy_pose, theta)

        # initialize vectors
        num_neighbors = 0
        COM     = np.array([0.,0.])  # center of mass
        v_align = np.array([0.,0.])  # alignment vector
        v_coh   = np.array([0.,0.])  # cohesion vector pointing to COM of all robots
        v_sep   = np.array([0.,0.])  # separation vector

        if len(self._agent_dict.keys()) > 0:
            """ iterate through possible neighbors (from messages) """
            for other_agent_name in self._agent_dict.keys():
                # compute distance to robot in message
                robot_vec = getVec(self._agent_dict[other_agent_name]['rx_state'].copy(), xy_pose, theta)
                angle_robot,dist_robot = getAngleAndDistance(robot_vec)
                angle_robot = wrap(angle_robot)
                """ only process if in comm range """
                if dist_robot < self._thresh_comm:
                    num_neighbors += 1

                    # send message about angle to other agents
                    msg = FlockingRelativeAngle(self._agent_name, other_agent_name, angle_robot)
                    self._agent_angle_pub.publish(msg)

                    ## calculate angle diff -- for alignment vector (if got rx_angle message)
                    if self._agent_dict[other_agent_name].has_key('rx_angle'):
                        if not(self._agent_dict[other_agent_name]['rx_angle']==0.):
                            angle_diff = self._agent_dict[other_agent_name]['rx_angle']-angle_robot-np.pi # should differ by pi when aligned
                            angle_diff = wrap(angle_diff)
                            v_align += getVec(np.array([1.,0.]), np.zeros(2), angle_diff)

                    ## sum vectors from running list -- for cohesion vector
                    COM += self._agent_dict[other_agent_name]['rx_state'].copy()

                    ## compute separation vector
                    dv_sep = R(np.pi).dot(robot_vec)
                    dist_sep = np.linalg.norm(dv_sep)
                    if dist_sep < self._thresh_sep:
                        v_sep += dv_sep

            """ Calculate coherence vector using center of mass (COM) in robot frame """
            if num_neighbors > 0:
                v_coh = getVec(COM, xy_pose, theta)
                v_coh = normalize(v_coh)
            """ normalize vectors """
            v_align = normalize(v_align)
            v_sep = normalize(v_sep)

        """ move based on combined vector angle (in robot frame) """
        v_combo = (v_migr*self._weight_migr + v_align*self._weight_align +
                    v_coh*self._weight_coh + v_sep *self._weight_sep)
        angle_combo, _ = getAngleAndDistance(v_combo)

        # modify if > max turn rate
        if angle_combo > self._thresh_turn:
            v_combo = normalize(v_combo)+np.array([0.1,0])
        elif angle_combo < -self._thresh_turn:
            v_combo = normalize(v_combo)-np.array([0.1,0])

        """ convert back to world frame """
        action = R(-theta).dot(v_combo)
        """ convert from velocity to accelleration """
        action = normalize(action,final=True) - vel

        """ remove agents that changed teams during iteration """
        for name in self.agents_to_remove:
            if self._agent_dict.has_key(name):
                self._agent_dict.pop(name)
        self.agents_to_remove = []

        return action

    def reset_callback(self,data):
        self.reset()
