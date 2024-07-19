from filterpy.kalman import UnscentedKalmanFilter as UKF
import numpy as np
from filterpy.common import Q_discrete_white_noise
from numpy.random import randn
from filterpy.kalman.sigma_points import MerweScaledSigmaPoints as merwe_sigma
from mercator_converter import *
import numpy as np
import numpy.linalg as lin

state_t_minus_one = np.empty((0, 3))
dt = 5.0

class Filter:

    def __init__(self, node_names, nodes):
        self.ukfs = {}
        for n in node_names:
            sigmas = merwe_sigma(4, alpha=1e-3, beta=2., kappa=1.)
            self.ukfs[n] = UKF(dim_x=4, dim_z=2, dt = dt, fx=state_transition_function, hx=measurement_function, points=sigmas)
    
    def initial_state(self, nodes): 
        for n in self.ukfs: 
            for node in nodes: 

                if n == node['associatedRouterName']: 
                    initial_lat = node['new_lat']
                    initial_lng = node['new_lng']
                    
                    [x0, y0] = toCartesian(initial_lat, initial_lng)
                    self.ukfs[node['associatedRouterName']].x = np.array([x0, 0, y0, 0])
                    self.ukfs[node['associatedRouterName']].Q = np.diag([node['new_variance'], 0.5, node['new_variance'], 0.5])
                    
    def apply_filter(self, nodes):

        # for each node, ukf
        for node in nodes: 
            unscented_kalman(self.ukfs[node['associatedRouterName']], node)
        

def state_transition_function(x, dt):
    """ state transition function for a static object """ 

    F = np.array([[1, dt, 0, 0], 
                  [0, 1, 0, 0], 
                  [0, 0, 1, dt], 
                  [0, 0, 0, 1]])

    return F @ x

def measurement_function(x): 
    """ measurement function is the output of the Least Square Lateration of RSSI values """

    return x[[0,2]]

Q_scale_factor = 10000
eps_max = 5
count = 0

def unscented_kalman(ukf, node):
    global Q_scale_factor, eps_max, count
    
    ukf.R = np.diag([node['new_variance'], node['new_variance']])
    #ukf.Q = Q_discrete_white_noise(dim=4, dt=dt, var=0.5)
    ukf.predict()
    [x_measurement, y_measurement] = toCartesian(node['new_lat'], node['new_lng'])
    z = [x_measurement, y_measurement]
    ukf.update(z)

    y, S = ukf.y, ukf.S
    eps = y.T @ lin.inv(S) @ y

    if eps > eps_max: 
        ukf.Q *= Q_scale_factor
        count += 1
    elif count > 0: 
        ukf.Q /= Q_scale_factor
        count -= 1


    #print("NEW Q: ", node['associatedRouterName'], ukf.Q)
    [node['new_lat'], node['new_lng']] = toGPS(ukf.x[0], ukf.x[2])

    P = ukf.P

    indices = [(0,0), (2,0), (0,2), (2,2)]

    submatrix = np.array([P[i, j] for i,j in indices]).reshape(2,2)

    vals, vecs = np.linalg.eigh(submatrix)

    # Sort eigenvalues and eigenvectors in descending order
    idx = vals.argsort()[::-1]
    eigenvalues = vals[idx]
    eigenvectors = vecs[:, idx]


    # Select the eigenvectors corresponding to the largest and second largest eigenvalues
    a = np.sqrt(eigenvalues[0])  # First column
    b = np.sqrt(eigenvalues[1]) # Third column
    
    # Get the rotation angle of the ellipse
    theta = np.degrees(np.arctan2(eigenvectors[1,0], eigenvectors[0,0]))

    node['theta_acc'] = float(theta)
    node['width_acc'] = float(a)
    node['height_acc'] = float(b)
    

