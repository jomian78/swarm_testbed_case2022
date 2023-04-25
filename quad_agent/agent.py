import sys
sys.path.append('../')

import rospy
import tf

from d_erg_lib import DErgControl
from d_erg_lib.utils import convert_ck2dist
from model import Model
from flocking_lib import DFlockingControl
from game_utils import GameUtils

from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension, String
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
import tf
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Empty
import os
import numpy as np
from datetime import datetime


class Agent(Model):
    def __init__(self, agent_name, id_number, num_agents):

        self.agent_name = agent_name
        self.agent_id = id_number
        self.num_agents = num_agents

        if (self.agent_id < (self.num_agents/2)): # if the id is from 0->num_agents/2, then the agent is red; else the agent is blue
            self.agent_team = "red"
        else:
            self.agent_team = "blue"
        rospy.Subscriber('/agent_team_status', String, self.team_callback)

        print('Agent name: ')
        print(self.agent_name)
        print('Agent id: ')
        print(self.agent_id)

        rospy.init_node(agent_name)

        self._rate = rospy.Rate(10)

        Model.__init__(self, self.agent_id, self.num_agents)
        num_pts = 50.0
        self.ctrllr = DErgControl(self.agent_name, self.agent_team, Model(self.agent_id, self.num_agents), num_basis=10, num_pts=num_pts, batch_size=100)
        """ added for flocking """
        self.ctrllr2 = DFlockingControl(self.agent_name, self.agent_team)
        self.game_utils = GameUtils(self.agent_name, self.agent_team, Model(self.agent_id, self.num_agents), num_basis=10, batch_size=100)

        self.__br = tf.TransformBroadcaster()

        #self._target_dist_pub = rospy.Publisher(self.agent_name+'/target_dist', GridMap, queue_size=1)

        gridmap = GridMap()
        arr = Float32MultiArray()
        arr.data = self.ctrllr._targ_dist.grid_vals[::-1]
        arr.layout.dim.append(MultiArrayDimension())
        arr.layout.dim.append(MultiArrayDimension())

        arr.layout.dim[0].label="column_index"
        arr.layout.dim[0].size=50
        arr.layout.dim[0].stride=50*50

        arr.layout.dim[1].label="row_index"
        arr.layout.dim[1].size=50
        arr.layout.dim[1].stride=50

        gridmap.layers.append("elevation")
        gridmap.data.append(arr)
        gridmap.info.length_x=10
        gridmap.info.length_y=10

        gridmap.info.pose.position.x=5
        gridmap.info.pose.position.y=5

        gridmap.info.header.frame_id = "world"
        gridmap.info.resolution = 0.2 # this is linked to the num_points was 0.2 for 10

        self._grid_msg = gridmap

        self.traj = [['x','y']]
        rospy.Subscriber(self.agent_name+'/save_traj', String, self.save) # set up subscriber to save traj data for only one agent
        rospy.Subscriber('save_all_traj', String, self.save) # set up subscriber to save traj data for all agents

        # publish blue/red team targets if this is agent0
        if (self.agent_name == 'agent0'):
            self.blue_gridmap = GridMap()
            blue_arr = Float32MultiArray()
            blue_arr.data = self.ctrllr._targ_dist.grid_vals[::-1]
            blue_arr.layout.dim.append(MultiArrayDimension())
            blue_arr.layout.dim.append(MultiArrayDimension())
            blue_arr.layout.dim[0].label="column_index"
            blue_arr.layout.dim[0].size=50
            blue_arr.layout.dim[0].stride=50*50
            blue_arr.layout.dim[1].label="row_index"
            blue_arr.layout.dim[1].size=50
            blue_arr.layout.dim[1].stride=50
            self.blue_gridmap.layers.append("elevation")
            self.blue_gridmap.data.append(blue_arr)
            self.blue_gridmap.info.length_x=10
            self.blue_gridmap.info.length_y=10
            self.blue_gridmap.info.pose.position.x=5
            self.blue_gridmap.info.pose.position.y=5
            self.blue_gridmap.info.header.frame_id = "world"
            self.blue_gridmap.info.resolution = 0.2 # this is linked to the num_points was 0.2 for 10

            self.red_gridmap = GridMap()
            red_arr = Float32MultiArray()
            red_arr.data = self.ctrllr._targ_dist.other_team_grid_vals[::-1]
            red_arr.layout.dim.append(MultiArrayDimension())
            red_arr.layout.dim.append(MultiArrayDimension())
            red_arr.layout.dim[0].label="column_index"
            red_arr.layout.dim[0].size=50
            red_arr.layout.dim[0].stride=50*50
            red_arr.layout.dim[1].label="row_index"
            red_arr.layout.dim[1].size=50
            red_arr.layout.dim[1].stride=50
            self.red_gridmap.layers.append("elevation")
            self.red_gridmap.data.append(red_arr)
            self.red_gridmap.info.length_x=10
            self.red_gridmap.info.length_y=10
            self.red_gridmap.info.pose.position.x=5
            self.red_gridmap.info.pose.position.y=5
            self.red_gridmap.info.header.frame_id = "world"
            self.red_gridmap.info.resolution = 0.2 # this is linked to the num_points was 0.2 for 10
            self.blue_team_pub = rospy.Publisher('/blue_team_target_dist', GridMap, queue_size=1)
            self.red_team_pub = rospy.Publisher('/red_team_target_dist', GridMap, queue_size=1)

        rospy.Subscriber('blue_connection_service', Empty, self.blue_connection_handle)
        rospy.Subscriber('red_connection_service', Empty, self.red_connection_handle)
        self.is_red_connected = False
        self.is_blue_connected = False

        # receiving kernel averages
        #self.blue_kernel_avg = None
        #self.red_kernel_avg = None
        #rospy.Subscriber('/blue_ck_plot', Float32MultiArray, self.blue_ck_callback)
        #rospy.Subscriber('/red_ck_plot', Float32MultiArray, self.red_ck_callback)

        #self.listener = tf.TransformListener()

    # def publish_team(self):
    #     team_msg = String()
    #     team_msg.data = "%s %s" % (self.agent_name, self.agent_team)
    #     self.team_pub.publish(team_msg)

    def team_callback(self, msg):
        """ Receive the updated team for this agent from the team_switch node. """
        msg_str = msg.data
        msg_str_list = list(msg_str.split(" "))
        if self.agent_name == msg_str_list[0]:
            self.agent_team = msg_str_list[1]

    def red_connection_handle(self, msg):
        """ Subscriber for a message (sent from the touchscreen) confirming the red player has connected and can see the testbed environment in Rviz. """
        self.is_red_connected = True
        rospy.logwarn("%s confirmed Red Player connected", self.agent_name)

    def blue_connection_handle(self, msg):
        """ Subscriber for a message (sent from the touchscreen) confirming the blue player has connected and can see the testbed environment in Rviz. """
        self.is_blue_connected = True
        rospy.logwarn("%s confirmed Blue Player connected", self.agent_name)

    def step(self):
        if self.agent_team == 'blue':
            ctrl = self.ctrllr(self.state) # this calculates the *next* control needed
        else:
            ctrl = self.ctrllr2(self.state) # this calculates the *next* control needed
            self.game_utils(self.state) # this send ck info to game engine
            rospy.logwarn_once("flocking")
            # rospy.logwarn("flocking {} | {}".format(self.state,ctrl))
        # pred_path = self.ctrllr.pred_path # might not be used
        super(Agent, self).step(ctrl) # this applies the control step to the model in model.py

        self.traj.append([self.state[0], self.state[1]])

        #num_tsteps = len(self.traj) -1
        # if self.agent_name == 'agent0':
        #     if num_tsteps % 50 == 0:
        #         print('tstep {}'.format(num_tsteps))

        self.__br.sendTransform( # sends update for rendering in RVIZ (or communication to BBN?)
            (self.state[0]*10, self.state[1]*10, 0.),
            (0.,0.,0.,1.),
            rospy.Time.now(),
            self.agent_name,
            "world"
        )

        # grid_vals =  self.ctrllr._targ_dist.grid_vals # convert_ck2dist(self.ctrllr._basis, self.ctrllr._ck_mean)
        # grid_vals = grid_vals * 10**6                  # we're only scaling the values that are sent to rviz - this is so we can see the target in rviz
        # self._grid_msg.data[0].data = grid_vals[::-1]
        # self._target_dist_pub.publish(self._grid_msg)

        if (self.agent_name == "agent0"):
            if (self.agent_team == "blue"):
                blue_vals = self.ctrllr._targ_dist.grid_vals
                blue_vals = blue_vals * 10**6
                self.blue_gridmap.data[0].data = blue_vals[::-1]
                self.blue_team_pub.publish(self.blue_gridmap)

                red_vals =  self.ctrllr._targ_dist.other_team_grid_vals
                red_vals = red_vals * 10**6
                self.red_gridmap.data[0].data = red_vals[::-1]
                self.red_team_pub.publish(self.red_gridmap)

            else:
                red_vals = self.ctrllr._targ_dist.grid_vals
                red_vals = red_vals * 10**6
                self.red_gridmap.data[0].data = red_vals[::-1]
                self.red_team_pub.publish(self.red_gridmap)

                blue_vals = self.ctrllr._targ_dist.other_team_grid_vals
                blue_vals = blue_vals * 10**6
                self.blue_gridmap.data[0].data = blue_vals[::-1]
                self.blue_team_pub.publish(self.blue_gridmap)

    def save(self,data):
        timestamp = datetime.now().strftime("_%Y%m%d%H%M%S")
        # read in path
        path = os.path.dirname(os.path.abspath(__file__))
        file_name = "../data/" + self.agent_name + '_' + data.data + timestamp + '.csv'
        file_path = os.path.join(path ,file_name)
        # save trajectory
        np.savetxt(file_path,self.traj,fmt="%s")
        # print status
        num_tsteps = len(self.traj) -1
        print('data for ' + self.agent_name + ' saved, {} time steps'.format(num_tsteps))

    def run(self):
        rospy.sleep(10)

        #initially take one step (before both players have connected) so the agents show up in rviz
        for _ in range(1):
            self._rate.sleep()
            self.step()

        while not rospy.is_shutdown():
            if (self.is_red_connected and self.is_blue_connected):
                self.step()
            self._rate.sleep()
