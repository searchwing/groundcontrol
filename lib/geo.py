# -*- coding: utf-8 -*-
from math import radians, sin, cos, sqrt, asin
 



class Position:

    def __init__(self, lat, lon, alt = None):
        self.lat, self.lon, self.alt = lat, lon, alt

    def distance(self, pos):
        return haversine(self.lat, self.lon, pos.lat, pos.lon) * 1000

    def __str__(self):
        return '%3.5f %3.5f %4i' % (self.lat, self.lon, self.alt)




def haversine(lat1, lon1, lat2, lon2):
    """Spherial distance of two lat/lon geo points.
    https://rosettacode.org/wiki/Haversine_formula#Python
    """
 
    R = 6372.8 # Earth radius in kilometers

    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
    c = 2*asin(sqrt(a))

    return R * c

