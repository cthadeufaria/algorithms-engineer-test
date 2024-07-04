import matplotlib.pyplot as plt
import numpy as np
from numpy import *
from numpy.linalg import inv
import pandas as pd


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
    # P = P - K @ IS @ K.T
    P = P - K @ H @ P
    return (X,P)
#
#

sensor_data = pd.read_csv('resources/sensor_data.csv')

for sensor in unique(sensor_data['sensor']):
    # ini state
    X = np.array( [ [0.0] , [0.0]] )
    # ini Covar
    P = np.array( [ [ 999.0, 0.0 ] ,
    [ 0.0, 999.0 ] ] )
    # meas matrix
    H = np.array( [ [ 1.0, 0.0 ] ] )
    # meas noise
    R = np.array([ [1.0] ] )
    # process noise
    Q = np.array(np.eye(2) * 5 )
    # meas
    Y = np.array([ [0.] ])

    # For Pretty Plotting
    t=0
    tx = []
    dx = [] # position of train over time (mean)
    vx = [] # velocitx of train over time (mean)
    dx_sd = [] # position of train over time (std_dev)
    vx_sd = [] # velocitx of train over time (std_dev)
    dx_up = [] # d mean + one std_dev
    dx_dn = [] # d mean - one std_dev
    vx_up = [] # v mean + one std_dev
    vx_dn = [] # v mean - one std_dev
    #

    Y = np.array([[0.]])

    data = sensor_data[sensor_data['sensor']==sensor]


    dt = 0.02;
    for i in range(0, max(data['sample_index'])):
        t += dt;

        U = np.array([[data['acc'][data['sample_index']==i].values[0]]])
        A = np.array( [ [ 1.0, dt * data['vec_x'][data['sample_index']==i].values[0]], [ 0.0, 1.0 ] ] )
        B = np.array( [ [(0.5*dt**2) * (data['vec_x'][data['sample_index']==i].values[0])], [dt] ] )

        (X, P) = kf_predict(X, P, A, Q, B, U)
        (X, P) = kf_update(X, P, Y, H, R)

        H = np.array([[ 1.0, 0.]])
        Y = H @ (A @ X + B @ U)

        tx.append(t)
        dx.append( X[0].item() )
        vx.append( X[1].item() )
        dx_sd.append( sqrt( P[0][0]).item() )
        vx_sd.append( sqrt( P[1][1]).item() )
        dx_up.append( X[0].item() + sqrt( P[0][0]).item() )
        dx_dn.append( X[0].item() - sqrt( P[0][0]).item() )
        vx_up.append( X[1].item() + sqrt( P[1][1]).item() )
        vx_dn.append( X[1].item() - sqrt( P[1][1]).item() )

    # ini state
    X = np.array( [ [0.0] , [0.0]] )
    # ini Covar
    P = np.array( [ [ 999.0, 0.0 ] ,
    [ 0.0, 999.0 ] ] )
    # meas matrix
    H = np.array( [ [ 1.0, 0.0 ] ] )
    # meas noise
    R = np.array([ [1.0] ] )
    # process noise
    Q = np.array(np.eye(2) * 5 )
    # meas
    Y = np.array([ [0.] ])

    t=0
    ty = []
    dy = [] # position of train over time (mean)
    vy = [] # velocity of train over time (mean)
    dy_sd = [] # position of train over time (std_dev)
    vy_sd = [] # velocity of train over time (std_dev)
    dy_up = [] # d mean + one std_dev
    dy_dn = [] # d mean - one std_dev
    vy_up = [] # v mean + one std_dev
    vy_dn = [] # v mean - one std_dev


    dt = 0.02;
    for i in range(0, max(data['sample_index'])):
        t += dt;

        U = np.array([[data['acc'][data['sample_index']==i].values[0]]])
        A = np.array( [ [ 1.0, dt * -data['vec_z'][data['sample_index']==i].values[0]], [ 0.0, 1.0 ] ] )
        B = np.array( [ [(0.5*dt**2) * (-data['vec_z'][data['sample_index']==i].values[0])], [dt] ] )

        (X, P) = kf_predict(X, P, A, Q, B, U)
        (X, P) = kf_update(X, P, Y, H, R)

        H = np.array([[ 1.0, 0.]])
        Y = H @ (A @ X + B @ U)

        ty.append(t)
        dy.append( X[0].item() )
        vy.append( X[1].item() )
        dy_sd.append( sqrt( P[0][0]).item() )
        vy_sd.append( sqrt( P[1][1]).item() )
        dy_up.append( X[0].item() + sqrt( P[0][0]).item() )
        dy_dn.append( X[0].item() - sqrt( P[0][0]).item() )
        vy_up.append( X[1].item() + sqrt( P[1][1]).item() )
        vy_dn.append( X[1].item() - sqrt( P[1][1]).item() )

    plt.scatter(dx, dy)
    plt.savefig(str(sensor) + '.png')