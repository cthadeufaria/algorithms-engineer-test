import matplotlib.pyplot as plt
import numpy as np
from numpy import dot
from numpy import *
from numpy.linalg import inv
from numpy.linalg import det
import random
random.seed(3)

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
#
#
# Initial Values
#
#
PI = np.pi
#Inter sample time
dt = 0.1;
# ini state
X = np.array( [ [0.0] , [0.0]] )
# ini Covar
P = np.array( [ [ 999.0, 0.0 ] ,
[ 0.0, 999.0 ] ] )
# state matrix
A = np.array( [ [ 1.0, dt ], [ 0.0, 1.0 ] ] )
# input effect matrix
B = np.array( [ [0.5*dt**2], [dt] ] )
# meas matrix
H = np.array( [ [ 1.0, 0.0 ] ] )
# meas noise
R = np.array([ [1.0] ] )
# process noise
Q = np.array(np.eye(2) * 5 )
# meas
Y = np.array([ [0.1] ])

# For Pretty Plotting
t=0
t_time = []
d_time = [] # position of train over time (mean)
v_time = [] # velocity of train over time (mean)
d_sd_time = [] # position of train over time (std_dev)
v_sd_time = [] # velocity of train over time (std_dev)
d_up_time = [] # d mean + one std_dev
d_dn_time = [] # d mean - one std_dev
v_up_time = [] # v mean + one std_dev
v_dn_time = [] # v mean - one std_dev
train_d = 0 # real train position
train_v = 0 # real train velocity
train_a = 0 # real train acceleration (this is the input of the system)
train_d_time = [] # real train position
train_v_time = [] # real train velocity
train_a_time = [] # real train acceleration (this is the input of the system)
#
#
# "Simulation" + Kalman Filter loop
#
#
N_iter = 50 # implies dt*N_iter seconds
for i in arange(0, N_iter):
    t += dt;

    if t<1:
        train_a=5 # acceleration
        R = np.array([ [1.0] ] ) # meas noise
    elif t<2:
        train_a=-4.5 # acceleration
        R = np.array([ [1000.0] ] ) # meas noise
    elif t<4:
        train_a=3 # acceleration
        R = np.array([ [1.0] ] ) # meas noise
    else:
        train_a=0 # acceleration
        R = np.array([ [50.0] ] ) # meas noise
    U = np.array([ [train_a] ]) # put the input in the right variable
    train_d += train_v * dt # real train position
    train_v += train_a * dt # real train velocity
    train_d_time.append(train_d) # real train position
    train_v_time.append(train_v) # real train velocity
    train_a_time.append(train_a) # real train acceleration
    Y = np.array([ [train_d * random.randrange(80, 120)/100 ] ]) # measurements are real +/- 20%
    (X, P) = kf_predict(X, P, A, Q, B, U)
    (X, P) = kf_update(X, P, Y, H, R)
    #print(X)
    t_time.append(t)
    d_time.append( X[0].item() )
    v_time.append( X[1].item() )
    d_sd_time.append( sqrt( P[0][0]).item() )
    v_sd_time.append( sqrt( P[1][1]).item() )
    d_up_time.append( X[0].item() + sqrt( P[0][0]).item() )
    d_dn_time.append( X[0].item() - sqrt( P[0][0]).item() )
    v_up_time.append( X[1].item() + sqrt( P[1][1]).item() )
    v_dn_time.append( X[1].item() - sqrt( P[1][1]).item() )


# End For Loop
fig = plt.figure(figsize=(8,8))
# d
chart1 = fig.add_subplot(211)
chart1.plot(t_time, train_d_time, label='train_d', c="b", linewidth=3, alpha=0.2)
chart1.plot(t_time,d_time, label='d', c="b")
chart1.fill_between(t_time, d_dn_time, d_up_time, alpha=0.2, linewidth=0, label='$d\pm\sigma$')
plt.legend(loc='upper left')
chart1.set_ylabel('d[m]')
plt.grid()
# v
chart2 = fig.add_subplot(212)
chart2.plot(t_time, train_v_time, label='train_v', c="g", linewidth=3, alpha=0.2)
chart2.plot(t_time,v_time, label='v', c="g")
chart2.fill_between(t_time,v_dn_time,v_up_time, alpha=0.2, label='$v\pm\sigma$')
chart2.set_ylabel('v [m/s]')
chart2.set_xlabel('t [s]')
plt.legend(loc='upper left')
plt.grid()
plt.show()