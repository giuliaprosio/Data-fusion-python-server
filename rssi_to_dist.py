# RSSI TO DISTANCE CONVERTER
import math 

MEASURED_POWER = -45 #-38
N = 1.9 #2.3


def rssiConversion(rssi): 
    """ Conversion of RSSI values into Distance in meters
    The empirical parameters are:  
    measured_power - the measured signal strength corresponding to 1m distance
    N - environmental factor to represent the level of obstruction in the building

    Even though there are typical ranges of values it is better to define said values 
    from empirical data collected. 

    """
    global MEASURED_POWER, N
   
    if(rssi is None): return None
    
    distance = 10 ** ((MEASURED_POWER - rssi) / (10 * N))
    
    return distance 