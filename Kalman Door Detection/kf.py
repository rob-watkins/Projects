import numpy as np, math
from scipy import stats
from scipy import signal
import scipy.stats


# kf_update: update state estimate [u, sigma] with new control [xdot] and measurement [z]
#   parameters:
#           u :         2x1 vector with state estimate (x) at time t-1 and control (xdot) at time t
#                       sigma: 2x2 matrix with covariance at time t-1
#           z (int):    observed (uncertain) measurement of state (x) at time t
#   returns: [u sigma] updated state with estimate at time t

def kf_update(u, sigma, z):
    F = np.matrix([[1, 1], [0, 1]])

    H = np.matrix([[1, 0], [0, 1]])

    R = np.diag([100.0, 100.0])

    Q = np.diag([0.003, 0.003])

    y = z - H * u

    S = H * sigma * H.T + R

    K = sigma * H.T * S.I

    u = u + K * y

    sigma = (H - K * H) * sigma

    sigma = F * sigma * F.T + Q

    u = F * u

    return [u, sigma]

# door_update: update estimate of door locations
#   parameters:
#           u :         2x1 vector with state estimate (x) at time t-1 and control (xdot) at time t-1
#                       sigma: 2x2 matrix with covariance at time t-1
#           doorSensor: door sensor at time t-1 
#                       door_dist (array of size 10): probability (0..1) that a door exists at each 
#                       location (0..9)
#   returns: [door_dist] updated door distribution

def door_update(u, sigma, doorSensor, door_dist):
    P_door = 0.5

    P_doorDetect = 0.6

    P_doorNoDetect = 0.4

    P_noDoorDetect = 0.2

    P_noDoorNoDetect = 0.8

    offset = min(int(u[0] / 100), 9)

    print sigma[(0, 0)]

    print u[0]

    print '------------------'

    P_doorExists = min(P_doorDetect * door_dist[offset] / (P_door * P_doorDetect + P_door * P_noDoorDetect), 1)

    P_doorDoesNotExist = P_doorNoDetect * door_dist[offset] / (P_door * P_doorNoDetect + P_door * P_noDoorNoDetect)

    if door_dist[offset] != 1:
        if doorSensor:
            door_dist[offset] = P_doorExists

        else:
            door_dist[offset] = P_doorDoesNotExist

    return door_dist
