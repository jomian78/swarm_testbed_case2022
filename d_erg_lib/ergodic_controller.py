import numpy as np
from basis import Basis
from barrier import Barrier
from replay_buffer import ReplayBuffer
from copy import deepcopy
import rospy
from std_msgs.msg import String, Float32MultiArray, Empty
from target_dist import TargetDist

from ergodic_humanswarmcollab_sim.msg import Ck

from matplotlib import pyplot as plt ## plotting for debugging
from utils import convert_ck2dist, convert_phik2phi

from collections import defaultdict ## used for creating a ck dict that has 3 values for each key value
"""
Decentralized ergodic controller for each agent. Takes in an agent identifer,
dynamics model, cost function weights, time horizon over which to calculate a trajectory,
number of basis functions to represent the trajectory in Fourier space,
a capacity for the replay buffer, and a batch size to sample from the replay
buffer when representing the trajectory of each agent. The controller outputs
a set of controls to be executed by the agent.

"""
class DErgControl(object):
    def __init__(self, agent_name, agent_team, model,
                    weights=None, t_horizon=10, num_basis=5, num_pts=50.,
                    capacity=100000, batch_size=20):

        self._agent_name    = agent_name
        
        self._agent_team    = agent_team
        rospy.Subscriber('/agent_team_status', String, self.team_callback)
        
        self._model         = model
        self._t_horizon     = t_horizon
        self._replay_buffer = ReplayBuffer(capacity)
        self._batch_size    = batch_size
        self._num_pts       = num_pts

        self._basis = Basis(self._model.explr_space, num_basis=num_basis)
        self._lamk  = np.exp(-0.8*np.linalg.norm(self._basis.k, axis=1))
        self._barr  = Barrier(self._model.explr_space)

        self._targ_dist = TargetDist(name=self._agent_name, team=self._agent_team, basis=self._basis, num_pts=num_pts)
 
        self._u = [0.0*np.zeros(self._model.action_space.shape[0])
                        for _ in range(t_horizon)]
        if weights is None:
            weights = {'R' : np.eye(self._model.action_space.shape[0])}
        self._Rinv = np.linalg.inv(weights['R'])

        self._phik          = None
        self._ck_mean       = None
        self._ck_msg        = Ck()
        self._ck_msg.name   = self._agent_name
        self._ck_msg.team   = self._agent_team
        #self._ck_dict       = {}
        self._ck_dict       = defaultdict(list)       #format: agent_name -> agent_team, ck, time_ck_was_received

        self.pred_path = []

        rospy.Subscriber('ck_link', Ck, self._ck_link_callback)
        self._ck_pub = rospy.Publisher('ck_link', Ck, queue_size=1)
        rospy.Subscriber('/clear_locs', Empty, self.reset_callback)

        # debugging
        # self.plot = False
        # if self.plot:
        #     plt.figure(figsize=[4,3])
        #     plt.ion()
        #     plt.title('pred_path {}'.format(agent_name))
        #     plt.show()

    def team_callback(self, msg):
        ''' receive the team this agent is on. '''
        msg_str = msg.data
        msg_str_list = list(msg_str.split(" "))
            
        if (self._agent_name == msg_str_list[0] and self._agent_team != msg_str_list[1]):
                self._agent_team = msg_str_list[1]
                self._replay_buffer.reset()
                #self._targ_dist.has_update = False

    def _ck_link_callback(self, msg):
        msg_timestamp = rospy.get_time()
        
        if msg.name != self._agent_name:
            if self._ck_dict.has_key(msg.name):
                self._ck_dict[msg.name][0] = msg.team
                self._ck_dict[msg.name][1] = np.array(msg.ck)
                self._ck_dict[msg.name][2] = msg_timestamp
            else:
                self._ck_dict.update({msg.name : [msg.team, np.array(msg.ck), msg_timestamp]})

    def reset(self):
        self._u = [0.0*np.zeros(self._model.action_space.shape[0])
                for _ in range(self._t_horizon)]
        self._replay_buffer.reset()


    def __call__(self, state):
        assert self._targ_dist.phik is not None, 'Forgot to set phik'

        if self._targ_dist.has_update==True:
            #self.reset()
            self._replay_buffer.reset()
            self._targ_dist.has_update = False

        self._u[:-1] = self._u[1:]
        self._u[-1]  = np.zeros(self._model.action_space.shape[0])

        x = self._model.reset(state.copy())

        pred_traj = []
        dfk       = []
        fdx       = []
        fdu       = []
        dbar      = []
        for t in range(self._t_horizon):

            # collect all the information that is needed
            pred_traj.append(
                    x[self._model.explr_idx]
            )
            dfk.append(
                    self._basis.dfk(x[self._model.explr_idx])
            )

            dbar.append(
                    self._barr.dx(x[self._model.explr_idx])
            )
            # step the model forwards
            #x = self._model.step(self._u[t] * 0.) -- why is this multiplied by 0?
            x = self._model.step(self._u[t])
            fdx.append(self._model.A)
            fdu.append(self._model.B)


        self.pred_path = deepcopy(pred_traj) # this might not be used anywhere

        # debugging
        # if self.plot:
        #     path = np.array(self.pred_path)
        #     plt.plot(path[:,0],path[:,1],'--')
        #     plt.draw()
        #     plt.pause(0.00000000001)
        # end debugging

        # sample any past experiences
        if len(self._replay_buffer) > self._batch_size:
            past_states     = self._replay_buffer.sample(self._batch_size)
            pred_traj       = pred_traj + past_states
        else:
            past_states = self._replay_buffer.sample(len(self._replay_buffer))
            pred_traj   = pred_traj + past_states

        # calculate the cks for the trajectory
        # *** this is also in the utils file
        N = len(pred_traj)
        ck = np.sum([self._basis.fk(xt) for xt in pred_traj], axis=0) / N
        #self._ck_msg.ck = ck.copy()
        #self._ck_msg.team = self._agent_team
        #self._ck_pub.publish(self._ck_msg)
        ck_msg = Ck()
        ck_msg.name = self._agent_name
        ck_msg.team = self._agent_team
        ck_msg.ck = ck.copy()
        self._ck_pub.publish(ck_msg)

        if len(self._ck_dict.keys()) > 1:
            curr_time = rospy.get_time()
            if self._ck_dict.has_key(self._agent_name):
                self._ck_dict[self._agent_name][0] = self._agent_team
                self._ck_dict[self._agent_name][1] = ck
                self._ck_dict[self._agent_name][2] = curr_time
            else:
                self._ck_dict.update({self._agent_name : [self._agent_team, ck, curr_time]})
            
            cks = []
            for key in self._ck_dict.keys():
                if (self._ck_dict[key][0] == self._agent_team):
                    cks.append(self._ck_dict[key][1])
            ck = np.mean(cks, axis=0)
            
        self._ck_mean = ck

        fourier_diff = self._lamk * (self._ck_mean - self._targ_dist.phik)
        fourier_diff = fourier_diff.reshape(-1,1)


        # backwards pass
        rho = np.zeros(self._model.observation_space.shape[0])
        for t in reversed(range(self._t_horizon)):
            edx = np.zeros(self._model.observation_space.shape[0])
            edx[self._model.explr_idx] = np.sum(dfk[t] * fourier_diff, 0)

            bdx = np.zeros(self._model.observation_space.shape[0])
            bdx[self._model.explr_idx] = dbar[t]
            rho = rho - self._model.dt * (-edx-bdx-np.dot(fdx[t].T, rho))

            #debugging
            # print('rho shape: ')
            # print(rho.shape)
            # print('edx shape: ')
            # print(edx.shape)
            # print('bdx shape: ')
            # print(bdx.shape)
            #print('fdx[t] shape: ')
            #print(fdx[t].shape)
            #print('fdx shape: ')
            #print(fdx)
            # print('self._u[t] shape:')
            # print(self._u[t].shape)
            # print('self._u[t]')
            # print(self._u[t])
            # print('self._u[t] * 0.')
            # print(self._u[t] * 0.)

            self._u[t] = -np.dot(np.dot(self._Rinv, fdu[t].T), rho)
            if (np.abs(self._u[t]) > 1.0).any():
                self._u[t] /= np.linalg.norm(self._u[t])
        self._replay_buffer.push(state[self._model.explr_idx].copy())
        return self._u[0].copy()

    def reset_callback(self,data):
        self.reset()
