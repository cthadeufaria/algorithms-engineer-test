import matplotlib.pyplot as plt
import numpy as np
from numpy import dot
from numpy import *
from numpy.linalg import inv
from numpy.linalg import det
import random
random.seed(3)


class KalmanFilter:
    """Class for Kalman Filter implementation."""
    def __init__(self,
                 dt=0.02):
        """Define initial state conditions."""
        self.dt = dt
        self.X = np.array([[0.0] , [0.0]])
        self.P = np.array([[999.0, 0.0] ,[0.0, 999.0]])
        self.H = np.array([[1.0, 0.0]])
        self.R = np.array([[1.0]])
        self.Q = np.array(np.eye(2)*5)
        self.Y = np.array([[0.]])
        self.time = 0.0
        self.coordinates = list()
        self.upper_bound = list()
        self.lower_bound = list()

    def update_state_space_system(self, vec_x, acc):
        """Update state space system."""
        self.time += self.dt
        self.U = np.array([[acc]])
        self.A = np.array([[1.0, self.dt*vec_x], [0.0, 1.0]])
        self.B = np.array([[(0.5*self.dt**2)*vec_x], [self.dt]])

    def update_measurement_estimate(self):
        self.Y = self.H @ (self.A @ self.X + self.B @ self.U)
    
    def predict(self):
        """
        X : The mean state estimate of the previous step (k-1) - shape(m,1)
        P : The state covariance of previous step (k-1) - shape(m,m)
        A : The transition matrix - shape(m,m)
        Q : The process noise covariance matrix - shape(m,m)
        B : The input effect matrix - shape(p, m)
        U : The control input - shape(q,1)
        """
        self.X = self.A @ self.X + self.B @ self.U
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self):
        """
        K : the Kalman Gain matrix
        IS : the Covariance or predictive mean of Y
        """
        IS = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ inv(IS)
        self.X = self.X + K @ (self.Y- self.H @ self.X)
        # self.P = self.P - K @ IS @ K.T
        self.P = self.P - K @ self.H @ self.P
    
    def append_coordinates(self):
        self.coordinates.append(self.X[0].item())
        self.upper_bound.append(self.X[0].item()+sqrt(self.P[0][0]).item())
        self.lower_bound.append(self.X[0].item()-sqrt(self.P[0][0]).item())

