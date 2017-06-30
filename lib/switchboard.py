# -*- coding: utf-8 -*-
"""Provide a GPS position relative to absolute local GPS position
actuated by a rotary encoder on the switchboard.
"""
import time

from . gps import gps
from . uav import uav
from . geo import Position
from . settings import *
from . serialthread import SerialThread


NAME = 'Board'
PORT = ROTARY_PORT
BAUD = ROTARY_BAUD




class Board(SerialThread):
    """Async (re)connect local serial switchboard,
     calculate a GPS position relative to absolute local GPS position.
    """

    def __init__(self, *args, **kwargs):
        super(Board, self).__init__(NAME, PORT, BAUD)
        self.arm, self.pos = False, None


    def iluminate(self):
        self.log('Iluminate')

        for code in (
                '1,0,0,0',
                '0,1,0,0',
                '0,0,1,0',
                '0,0,0,1',
                '1,0,0,0',
                '2,1,0,0',
                '2,2,1,0',
                '2,2,2,1',
                '2,2,2,2',
                '1,1,1,1',):
            self.ser.write('%s\r' % code)
            time.sleep(0.2)


    def work(self):
        self.iluminate()

        while 1:
            if self.pos:
                break
            self.log('Looking for local position')
            self.pos = gps.get_position()
            if self.pos:
                self.log('Got a local position')
            else:
                self.log('No local position')
                time.sleep(1)

        is_lat = True
        while 1:
            line = self.ser.readline().strip()
            if not line:
                continue

            try:
                enable, offs, direction, arm, launch, abort = map(int, line.split(','))
            except ValueError:
                continue
            arm, launch, abort = bool(arm), bool(launch), bool(abort)
            self.log('enable', enable, 'offs', offs, 'direction', direction, 'arm', arm, 'launch', launch, 'abort', abort)


            if not enable:
                self.ser.write("0,2,0,0\r")
                continue


            if direction:
                self.ser.write("2,0,0,0\r")
                self.log('Go from %s' % ('lon to lat' if is_lat else 'lat to lon',))
                is_lat = not is_lat


            if arm and not self.arm:
                if not self.pos:
                    self.log('No target yet')
                else:
                    uav.set_target(self.pos) and uav.prearm() and uav.arm()
            if not arm and self.arm:
                uav.disarm()
            self.arm = arm


            if launch:
                uav.launch()
            self.launch = launch


            if abort:
                uav.land()
                uav.disarm()


            self.ser.write("1,0,0,0\r")
            offs /= 1000000.0
            if is_lat:
                self.pos.lat += offs
            else:
                self.pos.lon += offs


            self.notify()


    def get_position(self):
        """Get calculated GPS position or None
        relative to absolute local GPS position.
        """
        return self.pos


board = Board()

