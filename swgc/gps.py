# -*- coding: utf-8 -*-
"""Provide current local GPS position from local serial GPS device.
"""
import time, datetime

from . import sync
from . import settings
from . geo import Position
from . serialthread import SerialThread




class GPS(SerialThread):
    """Async (re)connect local serial GPS device,
    parse NMEA, provide current local GPS position.
    Currently with 1hz.
    """

    def __init__(self, name, port, baud):
        super(GPS, self).__init__(name = name, port = port, baud = baud)
        self.position, self.ts, self.dt = None, None, None


    def work(self):
        while 1:
            sentence = self.ser.readline()[:-1]
            if sentence.startswith('$GPZDA'):
                sentence = sentence.split(',')
                clock, day, month, year = sentence[1:5]
                hour, minute, second = clock[:2], clock[2:4], clock[4:6]
                year, month, day, hour, minute, second = \
                    int(year), int(month), int(day), int(hour), int(minute), int(second)
                self.dt = datetime.datetime(year, month, day, hour, minute, second)
                self.ts = time.mktime(self.dt.timetuple())


            elif sentence.startswith('$GPGGA'):
                sentence = sentence.split(',')
                (frmat,
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
                 diff_ref_stationID) = sentence

                latitude = latitude.split('.')
                d,m = latitude
                d,m = d[:-2], d[-2:] + '.' + m
                latitude = int(d) + float(m) / 60

                longitude = longitude.split('.')
                d,m = longitude
                d,m = d[:-2], d[-2:] + '.' + m
                longitude = int(d) + float(m) / 60

                if northsouth == 'S':
                    latitude = -latitude
                if eastwest == 'W':
                    longitude = -longitude

                altitude = float(altitude)

                self.position = Position(lat = latitude, lon = longitude, alt = altitude)

                sync.notify()
                time.sleep(1) # GPS 1hz?


    def get_position(self):
        """Get last known position or None for
        latitude, longitude, altitude.
        Updated once per second.
        """
        return self.position


gps = GPS(
    name = 'GPS',
    port = settings.GPS_PORT, baud = settings.GPS_BAUD)
