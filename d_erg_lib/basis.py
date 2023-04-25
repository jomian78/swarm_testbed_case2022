import numpy as np
from numpy import pi, cos, sin
from scipy import integrate

class Basis(object):
    '''
    Cosine basis functions for decomposing distributions
    '''
    def __init__(self, explr_space, num_basis=5, offset=None):
        if offset is not None:
            raise NotImplementedError('Have not implemented offsets')
        self.dl  = explr_space[1,:] - explr_space[0,:]

        n = explr_space.shape[0]
        
        # k = np.meshgrid(*[[i for i in range(num_basis)] for _ in range(n)])
        k = np.meshgrid(*[range(num_basis) for _ in range(n)]) # equivalent to line above because 2D

        self.k = np.c_[k[0].ravel(), k[1].ravel()] # shape = (num_basis**2,2), not generalized for n > 2

        # hk wasn't actually being used. this implementation also didn't seem to work as expected? replaced with double integration from G. Matthew, I. Mezic, Metrics from ergodicity and design of ergodic dynamics from multi-agent systems, Physica D (2010)
        # self.hk = np.zeros(self.k.shape[0])
        #
        # for i, k in enumerate(self.k): # i = row number, k = row contents
        #     if np.prod(k) < 1e-5: # if product of row ~ 0 , element of hk = 1
        #         self.hk[i] = 1.
        #     else:
        #         top = np.prod(self.dl * (2.0 * k * np.pi + np.sin(2.0 * k *np.pi)) )
        #         bot = 16.0 * np.prod(k) * np.pi**2
        #         self.hk[i] = top/bot


        # hk from G. Matthew, I. Mezic, Metrics from ergodicity and design of ergodic dynamics from multi-agent systems, Physica D (2010)
        # note, this implementation is slow...
        # f = lambda x2,x1,k: np.prod(np.square(cos(k*np.array([x1,x2])*pi/self.dl)))
        # self.hk = np.zeros(self.k.shape[0])
        # for i, k in enumerate(self.k): # i = row number, k = row contents
        #     self.hk[i],_ = integrate.dblquad(lambda x1,x2: f(x1,x2,k),0,self.dl[1],lambda x1: 0, lambda x1: self.dl[0])

        self.tot_num_basis = num_basis**n

    def fk(self, x):
        """ x should have shape (batch, N)"""
        assert (x.shape[0] == self.dl.shape[0]), 'input dim does not match explr dim'
        return np.prod(np.cos(np.pi*x/self.dl * self.k),1)#/self.hk

    def dfk(self, x):
        # eq 20 from G. Matthew, I. Mezic, Metrics from ergodicity and design of ergodic dynamics from multi-agent systems, Physica D (2010)
         dx = np.zeros((self.tot_num_basis, x.shape[0]))
         dx[:,0] = -self.k[:,0]*pi*sin(pi * self.k[:,0] * x[0]/self.dl[0]) * cos(pi * self.k[:,1]*x[1]/self.dl[1])#/self.hk
         dx[:,1] = -self.k[:,1]*pi*sin(pi * self.k[:,1] * x[1]/self.dl[1]) * cos(pi * self.k[:,0]*x[0]/self.dl[0])#/self.hk
         return dx
