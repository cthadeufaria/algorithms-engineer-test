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
        self.X = np.array([[0.0] , [0.0]])
        self.P = np.array([[999.0, 0.0] ,[0.0, 999.0]])
        self.A = np.array([[1.0, dt], [0.0, 1.0]])
        self.B = np.array([[0.5*dt**2], [dt]])
        self.H = np.array([[1.0, 0.0]])
        self.R = np.array([[1.0]])
        self.Q = np.array(np.eye(2)*5)
        self.Y = np.array([[0.]])

    def kf_predict(X, P, A, Q, B, U):
        """
        X : The mean state estimate of the previous step (k-1) - shape(m,1)
        P : The state covariance of previous step (k-1) - shape(m,m)
        A : The transition matrix - shape(m,m)
        Q : The process noise covariance matrix - shape(m,m)
        B : The input effect matrix - shape(p, m)
        U : The control input - shape(q,1)
        """
        X = A @ X + B @ U
        P = A @ P @ A.T + Q
        return(X,P)

    def kf_update(X, P, Y, H, R):
        """
        K : the Kalman Gain matrix
        IS : the Covariance or predictive mean of Y
        """
        IS = H @ P @ H.T + R
        K = P @ H.T @ inv(IS)
        X = X + K @ (Y- H @ X)
        P = P - K @ IS @ K.T
        # P = P - K @ H @ P
        return (X,P)
    

