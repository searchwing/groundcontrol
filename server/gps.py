# -*- coding: utf-8 -*-
"""(Re)connect local GPS in threaded loop
Parse NMEA, provide current local position
"""
import time, serial, threading
from . settings import *




class _GPS(threading.Thread):
    """Internal only.
    """

    def __init__(self, *args, **kwargs):
        super(self.__class__, self).__init__(*args, **kwargs)
        self.longitude, self.latitude, self.altitude = None,  None, None
        self.timeOfFix = None

        self.daemon = True
        self.ser = None


    def run(self):
        while 1:
            try:
                while 1:
                    self._run()
            except serial.SerialException, e:
                print 'GPS serial failed', e
                self.ser = None
                time.sleep(2)


    def _run(self):
        if not self.ser:
            print 'Open GPS'

            self.ser = serial.Serial()
            self.ser.port     = GPS_PORT
            self.ser.baudrate = GPS_BAUD
            self.ser.timeout  = 10
            self.ser.open()

            print 'GPS open'

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

            timeOfFix = time.strftime("%H:%M:%S", time.strptime(utc.split(".")[0],"%H%M%S"))
            altitude = float(altitude)

            self.longitude, self.latitude, self.altitude = longitude, latitude, altitude
            self.timeOfFix = timeOfFix

            #print timeOfFix
            #print number_of_satellites_in_use
            #print altitude
            #print latitude, longitude


_gps = _GPS()


def start():
    """Start polling/connecting/parsing gps.
    """
    _gps.start()


def get_position():
    """Get latest known position or None.
    """
    return {
        'lat' : _gps.latitude,
        'lon' : _gps.longitude,
        'alt' : _gps.altitude,
    }

