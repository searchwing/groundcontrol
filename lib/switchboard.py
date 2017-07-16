# -*- coding: utf-8 -*-
"""Provide a GPS position relative to absolute local GPS position
actuated by a rotary encoder on the switchboard.
"""
import time

import ui
from . gps import gps
from . uav import uav
from . geo import Position
from . settings import *
from . serialthread import SerialThread




class Board(SerialThread):
    """Async (re)connect local serial switchboard,
     calculate a GPS position relative to absolute local GPS position.
    """

    def __init__(self, *args, **kwargs):
        super(Board, self).__init__('Board', BOARD_PORT, BOARD_BAUD)
        self.msg = None
        self.pos = None
        self.state = 0


    def iluminate(self):
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


    def m(self, msg):
        self.msg = msg
        self.log(self.msg)


    def get_message(self):
        return '(%s)\n%s' % (self.state, self.msg or '')


    def work(self):
        self.iluminate()
        while 1:
            try:
                self._work()
            except Exception, e:
                print e


    def _work(self):
        while 1:
            self.log('Looking for local position')
            self.pos = gps.get_position()
            ui.notify()
            if self.pos:
                self.log('Got a local position, start reading the board')
                break
            time.sleep(1)


        self.state = 1
        to_arm, to_disarm = False, False
        while 1:
            self.notify()

            line = self.ser.readline().strip()
            if not line:
                continue

            try:
                enable, offs, step, arm, trigger, abort = map(int, line.split(','))
            except ValueError:
                continue
            arm, trigger, abort = bool(arm), bool(trigger), bool(abort)
            #print 'enable', enable, 'offs', offs, 'step', step, 'arm', arm, 'trigger', trigger, 'abort', abort


            if not enable:
                self.m('Locked. Please turn key.')
                continue


            if to_arm and not arm:
                self.log('Waiting for arm')
                continue
            elif to_disarm and arm:
                self.log('Waiting for diarm')
                continue
            to_arm, to_disarm = False, False


            if step:
                self.state = (self.state + 1) % 5
                self.log('New state', self.state)


            if self.state == 1:
                if arm:
                    self.m('Please disarm.\nThen set latitude.')
                    to_disarm = True
                else:
                    self.m('Set latitude.')
                    if offs:
                        self.ser.write("1,0,0,0\r")
                        offs /= 1000000.0
                        self.pos.lat += offs


            if self.state == 2:
                if arm:
                    self.m('Please disarm.\nThen set longitude.')
                    to_disarm = True
                else:
                    self.m('Set longitude.')
                    if offs:
                        self.ser.write("1,0,0,0\r")
                        offs /= 1000000.0
                        self.pos.lon += offs


            if self.state == 3:
                if not arm:
                    self.m('Please arm.\nThen transmit target.')
                    to_arm = True
                else:
                    self.m('Transmit target.')
                    if trigger:
                        uav.set_target(self.pos)


            if self.state == 4:
                if not arm:
                    self.m('Pleased arm.\nThen launch.')
                    to_arm = True
                else:
                    self.m('Launch.')
                    if trigger:
                        uav.launch()


            elif abort:
                if not arm:
                    to_arm = True
                    self.m('Pleased arm.\nThen abort.')
                else:
                    uav.land()


            ui.notify()


    def get_position(self):
        """Get calculated GPS position or None
        relative to absolute local GPS position.
        """
        return self.pos


board = Board()

