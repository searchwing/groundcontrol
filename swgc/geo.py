# -*- coding: utf-8 -*-
"""A GEO Position class.
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
        return '%3.5f %3.5f %4i' % (
            self.lat, self.lon, self.alt)


    def __unicode__(self):
        return u'%3.5f %3.5f %4i' % (
            self.lat, self.lon, self.alt)


    @staticmethod
    def copy(pos):
        """Get Position object from passed object that
        has to have attributes lat, lon, alt.
        """
        return Position(
            pos.lat, pos.lon, pos.alt) if pos else None


    def get_distance(self, pos):
        """Get distance meters to passed position.
        """
        p1 = LatLon(self.lat, self.lon)
        p2 = LatLon(pos.lat, pos.lon)
        dist = p1.distance(p2) # km
        dist *= 1000 # m
        return dist


    def get_distance_and_heading(self, pos):
        """Get distance meters and heading degrees
        to passed position.
        """
        p1 = LatLon(self.lat, self.lon)
        p2 = LatLon(pos.lat, pos.lon)
        distance = p1.distance(p2) # km
        distance *= 1000 # m
        heading = p1.heading_initial(p2) # degree
        if heading < 0:
            heading += 360
        return distance, heading


    def get_location_by_offset_meters_and_heading(self, distance, heading):
        """Get Position in passed distance meters
        and heading degrees.
        """
        distance = geopy.distance.distance(meters = distance)
        destination = distance.destination(
            geopy.Point(self.lat, self.lon), heading)
        return Position(
            lat = destination.latitude,
            lon = destination.longitude,
            alt = self.alt)
