# RSSI TO DISTANCE CONVERTER
import math 

def rssiConversion(rssi): 
    """ Conversion of RSSI values into Distance in meters
    The empirical parameters are:  
    measured_power - the measured signal strength corresponding to 1m distance
    N - environmental factor to represent the level of obstruction in the building

    Even though there are typical ranges of values it is better to define said values 
    from empirical data collected. 

    """
    measured_power = -45 #-38 
    N = 1.9 #2.3
    if(rssi is None): return None
    #print("rssi value: ", rssi)
    #print("DISTANCE", 10**(measured_power - rssi)/(10*N))
    distance = 10 ** ((measured_power - rssi) / (10 * N))
    #print(distance)
    return distance #math.pow(10, (measured_power - rssi)/(10*N))