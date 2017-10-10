# -*- coding: utf-8 -*-
"""Some geo stuff.
"""
from math import degrees, atan2

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


    def get_distance(self, pos):
        """Get distance meters to passed position.
        """
        return geopy.distance.distance(
            (self.lat, self.lon), (pos.lat, pos.lon)).meters

    def get_bearing(self, pos):
        """Get bearing degrees to passed location.
        """
        return degrees(atan2(self.lat - pos.lat, self.lon - pos.lon))


    def get_location_by_offset_meters_and_bearing(self, distance, bearing):
        """Get Position in passed distance meters and bearing.
        """
        dist = geopy.distance.distance(meters = distance)
        dest = dist.destination(
            geopy.Point(self.lat, self.lon), bearing)
        return Position(
            lat = dest.latitude, lon = dest.longitude, alt = self.alt)


    @staticmethod
    def copy(pos):
        """Return Position opbject from passed object that
        has to have attributes lot, lat, lon.
        """
        return Position(pos.lat, pos.lon, pos.alt)
