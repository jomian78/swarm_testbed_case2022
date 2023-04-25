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


class SinglePlayerAgent(Model):
    def __init__(self, agent_name, id_number, num_agents, team):

        self.agent_name = agent_name
        self.agent_id = id_number
        self.num_agents = num_agents
        self.agent_team = team

        #rospy.Subscriber('/agent_team_status', String, self.team_callback)

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

        if (self.agent_name == "agent0"):
            self._target_dist_pub = rospy.Publisher(self.agent_team + '_target_dist', GridMap, queue_size=1)

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

        if self.agent_team == "blue":
            rospy.Subscriber('blue_connection_service', Empty, self.connection_handle)
        else:
            rospy.Subscriber('red_connection_service', Empty, self.connection_handle)
        self.is_connected = False

    # def team_callback(self, msg):
    #     """ Receive the updated team for this agent from the team_switch node. """
    #     msg_str = msg.data
    #     msg_str_list = list(msg_str.split(" "))
    #     if self.agent_name == msg_str_list[0]:
    #         self.agent_team = msg_str_list[1]

    def connection_handle(self, msg):
        """ Subscriber for a message (sent from the touchscreen) confirming the player has connected and can see the testbed environment in Rviz. """
        self.is_connected = True
        rospy.logwarn('%s confirmed single player connected', self.agent_name)

    def step(self):
        if self.agent_team == 'blue':
            ctrl = self.ctrllr(self.state) # this calculates the *next* control needed
        else:
            ctrl = self.ctrllr2(self.state) # this calculates the *next* control needed
            self.game_utils(self.state) # this send ck info to game engine
            # rospy.logwarn("flocking {} | {}".format(self.state,ctrl))
        # pred_path = self.ctrllr.pred_path # might not be used
        super(SinglePlayerAgent, self).step(ctrl) # this applies the control step to the model in model.py

        self.traj.append([self.state[0], self.state[1]])
        num_tsteps = len(self.traj) -1

        self.__br.sendTransform( # sends update for rendering in RVIZ (or communication to BBN?)
            (self.state[0]*10, self.state[1]*10, 0.),
            (0.,0.,0.,1.),
            rospy.Time.now(),
            self.agent_name,
            "world"
        )

        if (self.agent_name == "agent0"):
            grid_vals =  self.ctrllr._targ_dist.grid_vals * 10**6 # convert_ck2dist(self.ctrllr._basis, self.ctrllr._ck_mean) and scale the values up for viewing in rviz
            self._grid_msg.data[0].data = grid_vals[::-1]
            self._target_dist_pub.publish(self._grid_msg)

    def save(self,data):
        timestamp = datetime.now().strftime('_%Y%m%d%H%M%S')
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
            if (self.is_connected):
                self.step()
            self._rate.sleep()
