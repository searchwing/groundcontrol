# -*- coding: utf-8 -*-
"""Provide current local GPS position
from local serial GPS device.
"""
import time
from . geo import Position
from . settings import *
from . serialthread import SerialThread


NAME = 'GPS'
PORT = GPS_PORT
BAUD = GPS_BAUD




class GPS(SerialThread):
    """Async (re)connect local serial GPS device,
    parse NMEA, provide current local GPS position.
    """

    def __init__(self):
        super(GPS, self).__init__(NAME, PORT, BAUD)
        self.position = None


    def work(self):
        while 1:
            sentence = self.ser.readline()[:-1]
            if sentence.startswith('$GPGGA'):
                (format,
                 utc,
                 latitude,
                 northsouth,
                 longitude,
                 eastwest,
                 quality,
                 number_of_satellites_in_use,
                 horizontal_dilution,
                 altitude,
                 above_sea_unit,
                 geoidal_separation,
                 geoidal_separation_unit,
                 data_age,
                 diff_ref_stationID) = sentence.split(",")

                latitude, longitude = float(latitude)/100, float(longitude)/100

                if northsouth == 'S':
                    latitude = -latitude
                if eastwest == 'W':
                    longitude = -longitude
                altitude = float(altitude)

                self.position = Position(lat = latitude, lon = longitude, alt = altitude)

                time.sleep(1) # GPS 1hz?


    def get_position(self):
        """Get last known position or None for
        latitude, longitude, altitude.
        Updated once per second.
        """
        return self.position


gps = GPS()

