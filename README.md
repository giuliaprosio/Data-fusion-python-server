## Data Fusion Python Server
Server that receives data in a json struct with the following structure: 
'''
    nodes: [
        {
            ip: 
            latitude:
            longitude: 
            timestamp: 
            accuracy: 
        }
        ...
    ],
    edges: [
        {
            source: 
            destination: 
            rssi: 
        }
        ...
    ]
'''
And using a probabilistic approach uses the RSSI parameters to produce a probable distance between 
nodes. Using these relative distances and Least Square Lateration, each node finds a new position. This 
is fused with the nodes own GPS sensor data, accordingly to the covariances associated. 

Then, to smooth out errors, each node's probable position is passed through a 
smoothing filter, using Unscented Kalman Filters. The data at timestamp k-1 is 
used to predict the position of each node at timestamp k and this prediction is compared with the 
produced position from the sensors. 