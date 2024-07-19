import math 

R = 6371000

def toCartesian(latitude, longitude): 
    """
    To project latitude and longitude coordinates into Cartesian space, 
    make use of Mercator Conversion. 
    It can be used only when dealing with small distances, as it does not 
    take into account the curvature of the Earth. 

    Input: Latitude, Longitude
    Output: x, y
    """

    latitude_radians = latitude*math.pi/180
    longitude_radians = longitude*math.pi/180

    x = R*longitude_radians
    y = R*(math.log(math.tan((math.pi/4)+(latitude_radians/2))))

    return [x, y]


def toGPS(x, y): 
    """
    Inverse function using the inverse formula of Mercator Conversion in order to 
    retrieve latitude and longitude coordinates after the operations done in
    Cartesian space. 
    """
    latitude_radians = (2 * math.atan(math.pow(math.e, y / R)) - math.pi / 2)
    longitude_radians = (x/R)

    return [latitude_radians * 180 / math.pi, longitude_radians*180/math.pi] 

