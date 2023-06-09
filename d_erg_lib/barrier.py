import numpy as np

class Barrier(object):
    """
    This class prevents the agent from
    going outside the exploration space.
    """
    def __init__(self, explr_space, pow=2, weight=100): #original barrier strength was pow=2, original weight was weight=100; pow must be even
        self.explr_space = explr_space
        self.dl  = explr_space[1,:] - explr_space[0,:]
        self.pow = pow
        self.weight = weight

        self.eps = 0.01           
            
    def cost(self, x):
        """
        Returns the actual cost of the barrier.
        """
        cost = 0.
        cost += np.sum((x > self.explr_space[1,:]-self.eps) * (x - (self.explr_space[1,:]-self.eps))**self.pow)
        cost += np.sum((x < self.explr_space[0,:]+self.eps) * (x - (self.explr_space[0,:]+self.eps))**self.pow)
        return self.weight * cost

    def dx(self, x):
        """
        Returns the derivative of the barrier wrt to the exploration
        state.
        """
        dx = np.zeros(x.shape)
        dx += 2*(x > (self.explr_space[1,:]-self.eps)) * (x - (self.explr_space[1,:]-self.eps))
        dx += 2*(x < (self.explr_space[0,:]+self.eps)) * (x - (self.explr_space[0,:]+self.eps))

        return self.weight * dx
