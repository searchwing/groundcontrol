# -*- coding: utf-8 -*-
"""Provide a GPS position relative to absolute local GPS position
actuated by a rotary encoder on the switchboard.
"""
import time, threading

from . gps import gps
from . geo import Position
from . settings import *
from . serialthread import SerialThread


NAME = 'Rotary'
PORT = ROTARY_PORT
BAUD = ROTARY_BAUD




class Board(SerialThread):
    """Async (re)connect local serial switchboard,
     calculate a GPS position relative to absolute local GPS position.
    """

    def __init__(self, *args, **kwargs):
        super(Board, self).__init__(NAME, PORT, BAUD)
        self.cond = threading.Condition()
        self.pos = None
        self.is_lat = True


    def wait(self, timeout = None):
        self.cond.acquire()
        self.cond.wait(timeout = timeout)
        self.cond.release()


    def notify(self):
        self.cond.acquire()
        self.cond.notifyAll()
        self.cond.release()


    def work(self):

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

        while 1:
            line = self.ser.readline()
            if not line:
                continue
            line = line[:-2]

            offs, direction = line.split(',')
            offs, direction = int(offs), int(direction)

            if offs == 0:
                self.is_lat = not self.is_lat

            else:
                offs /= 1000000.0

                if self.is_lat:
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

