import numpy as np

class Model(object):
    def __init__(self, agent_id, num_agents):
        self.agent_id = agent_id
        self.num_agents = num_agents
        
        self.observation_space  = np.array([[0., 0., -np.inf, -np.inf],[1., 1., np.inf, np.inf]], dtype=np.float32).T
        self.action_space       = np.array([[-1., -1.],[+1., +1.]], dtype=np.float32)
        
        # self.explr_space        = np.array([[0., 0.],[4.50, 2.25]], dtype=np.float32)      # shelby
        #self.explr_space        = np.array([[0., 0.],[20., 10.]], dtype=np.float32)         # shelby

        self.explr_space        = np.array([[0., 0.],[1., 1.]], dtype=np.float32)            # default env
        self.explr_idx          = [0, 1]                                                     # specifies which elements are used for exploration (i.e x,y)

        self.dt = 0.1
        self._A = np.array([       # double integrator model
                [0., 0., 0.8, 0.], 
                [0., 0., 0., 0.8], 
                [0., 0., 0., 0.],  
                [0., 0., 0., 0.]   
        ])

        self._B = np.array([
                [0., 0.],
                [0., 0.],
                [1.0, 0.],
                [0., 1.0]
        ])
        

        self.reset()

    def reset(self, state=None):
        '''
        Resets the property self.state
        '''
        if state is None:
            # if (self.agent_id == 0 or self.agent_id == 4 or self.agent_id == 8 or self.agent_id == 12 or self.agent_id == 16 or self.agent_id == 20 or self.agent_id == 24 or self.agent_id == 28):
            #     self.state = np.array([0.1, 0.1, 0.1, 0.1])
            # elif (self.agent_id == 1 or self.agent_id == 5 or self.agent_id == 9 or self.agent_id == 13 or self.agent_id == 17 or self.agent_id == 21 or self.agent_id == 25 or self.agent_id == 29):
            #     self.state = np.array([0.9, 0.9, 0.1, 0.1])
            # elif (self.agent_id == 2 or self.agent_id == 6 or self.agent_id == 10 or self.agent_id == 14 or self.agent_id == 18 or self.agent_id == 22 or self.agent_id == 26 or self.agent_id == 30):
            #     self.state = np.array([0.1, 0.9, 0.1, 0.1])
            # else:
            #     self.state = np.array([0.9, 0.1, 0.1, 0.1])

            if (self.agent_id < (self.num_agents/2)): #spawn blue and red teams in opposite corners
                #self.state = np.array([0.1, 0.1, 0.1, 0.1])
                self.state = np.concatenate([np.random.uniform(0.1, 0.2, size=(2,)), np.random.uniform(0.0, 0.2, size=(2,))]) 
            else:
                #self.state = np.array([0.9, 0.9, 0.1, 0.1])
                self.state = np.concatenate([np.random.uniform(0.8, 0.9, size=(2,)), -np.random.uniform(0.0, 0.2, size=(2,))])
                
                
        else:
            self.state = state.copy()

        return self.state.copy()

    @property
    def state(self):
        return self._state.copy()
    @state.setter
    def state(self, x):
        self._state = x.copy()
    @property
    def A(self, x=None, u=None):
        return self._A.copy()
    @property
    def B(self, x=None, u=None):
        return self._B.copy()

    def f(self, x, u):
        '''
        Continuous time dynamics
        '''
        return np.dot(self._A, x) + np.dot(self._B, u)

    def step(self, a): # this is where `super` from agent.py goes
        state = self._state + self.f(self._state, a) * self.dt
        self._state = state
        return state.copy()
