#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
import math




def get_distance_metres(lat1, lon1, lat2, lon2):
    """Returns the ground distance in metres between two geo coordinates.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat, dlon = lat2 - lat1, lon2 - lon1
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5




def get_location_offset_meters(lat, lon, dNorth, dEast):
    """Returns a latitude/longitude containing `dNorth` and `dEast` metres from the passed 'lat'/'lon' latitude/longitude.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    #Radius of 'spherical' earth
    earth_radius = 6378137.0

    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast  / (earth_radius * math.cos(math.pi * lat / 180))

    #New position in decimal degrees
    newlat = lat + (dLat * 180/math.pi)
    newlon = lon + (dLon * 180/math.pi)

    return newlat, newlon

