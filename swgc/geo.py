# -*- coding: utf-8 -*-
"""Some geo stuff.
"""
from LatLon import LatLon
import geopy, geopy.distance




class Position(object):
    """Lat/Lon/Alt position and functions.
    """

    def __init__(self, lat, lon, alt):
        self.lat, self.lon, self.alt = lat, lon, alt


    def __str__(self):
        """To string.
        """
        return '%3.5f %3.5f %4i' % (self.lat, self.lon, self.alt)


    def __unicode__(self):
        return u'%3.5f %3.5f %4i' % (self.lat, self.lon, self.alt)


    @staticmethod
    def copy(pos):
        """Get Position object from passed object that
        has to have attributes lat, lon, alt.
        """
        return Position(pos.lat, pos.lon, pos.alt)


    def get_distance(self, pos):
        """Get distance meters to passed position.
        """
        p1 = LatLon(self.lat, self.lon)
        p2 = LatLon(pos.lat, pos.lon)
        dist = p1.distance(p2) # km
        dist *= 1000 # m
        return dist


    def get_distance_and_heading(self, pos):
        """Get distance meters and heading degrees to passed position.
        """
        p1 = LatLon(self.lat, self.lon)
        p2 = LatLon(pos.lat, pos.lon)
        dist = p1.distance(p2) # km
        dist *= 1000 # m
        head = p1.heading_initial(p2) # degree
        if head < 0:
            head += 360
        return dist, head


    def get_location_by_offset_meters_and_heading(self, distance, heading):
        """Get Position in passed distance meters and heading degrees.
        """
        dist = geopy.distance.distance(meters = distance)
        dest = dist.destination(
            geopy.Point(self.lat, self.lon), heading)
        return Position(
            lat = dest.latitude, lon = dest.longitude, alt = self.alt)
