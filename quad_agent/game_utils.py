import numpy as np
from copy import deepcopy
import rospy
from std_msgs.msg import String, Float32MultiArray, Empty

from ergodic_humanswarmcollab_sim.msg import Ck

import os
import sys
new_path = os.path.dirname(os.path.abspath(__file__))+"/../d_erg_lib"
sys.path.append(new_path)

from replay_buffer import ReplayBuffer
from basis import Basis

from collections import defaultdict ## used for creating a ck dict that has 3 values for each key value

"""
publishes "ck"s for the game for other tactics
"""

class GameUtils(object):
    def __init__(self, agent_name, agent_team, model,
                    num_basis=5, capacity=100000, batch_size=20):

        self._agent_name    = agent_name
        self._agent_team    = agent_team
        rospy.Subscriber('/agent_team_status', String, self.team_callback)

        self._model         = model
        self._replay_buffer = ReplayBuffer(capacity)
        self._batch_size    = batch_size

        self._basis = Basis(self._model.explr_space, num_basis=num_basis)
        self._ck_pub = rospy.Publisher('ck_link', Ck, queue_size=1)
        rospy.Subscriber('/clear_locs', Empty, self.reset_callback)

    def team_callback(self, msg):
        ''' receive the team this agent is on. '''
        msg_str = msg.data
        msg_str_list = list(msg_str.split(" "))

        if (self._agent_name == msg_str_list[0] and self._agent_team != msg_str_list[1]):
                self._agent_team = msg_str_list[1]
                self._replay_buffer.reset()

    def reset(self):
        self._replay_buffer.reset()

    def __call__(self, state):

        self._replay_buffer.push(state[self._model.explr_idx].copy())
        # sample any past experiences
        if len(self._replay_buffer) > self._batch_size:
            past_states = self._replay_buffer.sample(self._batch_size)
        else:
            past_states = self._replay_buffer.sample(len(self._replay_buffer))

        # calculate the cks for the trajectory
        # *** this is also in the utils file
        N = len(past_states)
        ck = np.sum([self._basis.fk(xt) for xt in past_states], axis=0) / N
        ck_msg = Ck()
        ck_msg.name = self._agent_name
        ck_msg.team = self._agent_team
        ck_msg.ck = ck.copy()
        self._ck_pub.publish(ck_msg)

    def reset_callback(self,data):
        self.reset()
