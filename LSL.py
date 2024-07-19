
import math
import numpy as np
from numpy.linalg import inv
from rssi_to_dist import *
from mercator_converter import *
from scipy.linalg import lstsq

# LEAST SQUARE LATERATION 
# Function taking as input a topology graph, and output
# for each node the new coordinates given by the Least 
# Square Lateration estimation

def lsl_supporter(graph, idx, node):
    """
    Receives reference to whole topology and the index of the node of which
    to calculate the LSL. 
    Outputs the new coordinates for said node using Least Square Lateration
    algorithm on its neighbouring nodes. 
    """
    
    matrixArray = np.empty((0, 3))

    sum_values = 0
    howmany = 0

    for otherIdx, otherNode in enumerate(graph['nodes']):
        
        if(otherIdx == idx): continue
        if(otherNode['lat'] == None or otherNode['lng'] == None): continue

        sum_values += otherNode['variance'] + 0.9999
        howmany += 1

        d1 = 0.0
        d2 = 0.0 

        for edge in graph['edges']: 
            if((edge['source'] == idx and edge['dest'] == otherIdx)): 
                d1 = edge['distance']
            elif((edge['source'] == otherIdx and edge['dest'] == idx)):
                d2 = edge['distance']
        
        d = max(d1, d2) 
       
        nodeDistanceArray = np.array([otherNode['mercator_x'], otherNode['mercator_y'], d])
        
        if None in nodeDistanceArray: 
            print("Skipping this as undefined element inside")
        else:
            nodeDistanceArray = nodeDistanceArray.reshape(1, -1)
            matrixArray = np.concatenate((matrixArray, nodeDistanceArray), axis = 0)

    # crearion of matrices A and B - both A and B  
    # A DIM N x 2
    # B DIM N x 1
    A = np.empty((0, 2))
    B = np.empty((0, 1))

    x0 = matrixArray[0][0]
    y0 = matrixArray[0][1]
    d0 = matrixArray[0][2]

    for z in range(1, len(matrixArray)): 
        xdiff = 2*(matrixArray[z][0] - x0)
        ydiff = 2*(matrixArray[z][1] - y0)

        diffRowA = np.array([xdiff, ydiff])
        diffRowA = diffRowA.reshape(1, -1)
        A = np.concatenate((A, diffRowA), axis = 0)

        rowB = d0**2 - matrixArray[z][2]**2 + matrixArray[z][0]**2 + matrixArray[z][1]**2 - x0**2 - y0**2
        diffRowB = np.array([rowB])
        diffRowB = diffRowB.reshape(1, -1)
        B = np.concatenate((B, diffRowB), axis = 0)  
    
    p, res, rnk, s = lstsq(A, B, lapack_driver='gelsy')

    xnew = 0.0
    ynew = 0.0

    [xnew, ynew] = [p[0], p[1]]
    

    latnew = 0.0
    lngnew = 0.0
    [latnew, lngnew] = toGPS(xnew, ynew)

    newposvariance = sum_values/howmany

    return [latnew, lngnew, newposvariance]



WORST_ACC = float('-inf')
BEST_ACC = float('inf') 

def lsl(graph): 
    """
    Prepares data for Least Square Lateration algorithm. For each node calculates
    the projected coordinates using Mercator Projection and assesses the best 
    and worst accuracies among the nodes to rank them in the final weighting.
    """

    nodes = graph['nodes']
    edges = graph['edges']

    global BEST_ACC, WORST_ACC
    
    # data preparation
    for node in nodes: 
        if node['lat'] == None or node['lng'] == None: continue
        if(node['associatedRouterName'] == "YOI-04" and node['acc'] > 20): node['acc'] = 20
        if node['acc'] < BEST_ACC: BEST_ACC = node['acc']
        if node['acc'] > WORST_ACC: WORST_ACC = node['acc']
        
        [node['mercator_x'], node['mercator_y']] = toCartesian(node['lat'], node['lng'])

        node['variance'] = (node['acc']**2)
    
    for edge in edges: 
        edge['distance'] = rssiConversion(edge['rssi']) 

    # least square lateration
    for idx, node in enumerate(nodes): 
        
        [new_lat, new_lng, new_var] = lsl_supporter(graph, idx, node)

        # having LSL coordinates, do an average of them and the gps coordinates proportional to the 
        # accuracy of the gps coordinates 
        node['new_variance'] = new_var

        if ( not node['lat'] == None and not node['lng'] == None): 

            # MIN-MAX scaling with adjusted range
            epsilon = 0.5  # A small positive value to avoid division by zero
            min_adjusted = WORST_ACC + epsilon
            max_adjusted = BEST_ACC - epsilon
            gps_weight = (node['acc'] - min_adjusted) / (max_adjusted - min_adjusted)
            lsl_weight = (1 - gps_weight)

            node['new_lat'] = float(new_lat)*lsl_weight + node['lat']*gps_weight
            node['new_lng'] = float(new_lng)*lsl_weight + node['lng']*gps_weight
            node['new_variance'] = new_var*lsl_weight + node['variance']*gps_weight 
  
        else: 
            
            node['new_lat'] = float(new_lat)
            node['new_lng'] = float(new_lng)
            node['new_variance'] = float(new_var)
        

            
             


    

