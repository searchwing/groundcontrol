# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
"""Some geo stuff.
"""
from math import pi, radians, sin, cos, asin, sqrt




class Position(object):
    """Lat/Lon/Alt position.
    """

    def __init__(self, lat, lon, alt = None):
        self.lat, self.lon, self.alt = lat, lon, alt

    def distance(self, pos):
        """Get distance to passed position.
        """
        return haversine(self.lat, self.lon, pos.lat, pos.lon) * 1000

    def __str__(self):
        """To string.
        """
        return '%3.5f %3.5f %4i' % (self.lat, self.lon, self.alt)

    def __unicode__(self):
        return u'%s' % self.__str__()

    @staticmethod
    def copy(pos):
        """Return Position opbject from passed object that
        has to have attributes lot, lat, lon.
        """
        return Position(pos.lat, pos.lon, pos.alt)




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




def get_location_offset_meters(lat, lon, dNorth, dEast):
    """Returns a latitude/longitude containing `dNorth` and `dEast` metres
    from passed 'lat'/'lon' latitude/longitude.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    #Radius of 'spherical' earth
    earth_radius = 6378137.0

    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast  / (earth_radius * cos(pi * lat / 180))

    #New position in decimal degrees
    newlat = lat + (dLat * 180/pi)
    newlon = lon + (dLon * 180/pi)

    return newlat, newlon
