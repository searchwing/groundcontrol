# -*- coding: utf-8 -*-
"""Provide a GPS position relative to absolute local GPS position
actuated by a rotary encoder on the switchboard.
"""
import time, traceback

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
                traceback.print_exc()


    def _work(self):
        while 1:
            #self.log('Looking for local position')
            self.pos = gps.get_position()
            ui.notify()
            if self.pos:
                self.log('Got a local position, start reading the board')
                break
            time.sleep(1)


        self.state = 0
        while 1:
            self.notify()


            if not uav.get_uav():
                self.state = 0
                while 1:
                    self.m('Waiting for UAV...')
                    self.notify()
                    time.sleep(2)

                    if uav.get_uav():
                        uav.set('ARMING_CHECK',     9)
                        uav.set('BRD_SAFETYENABLE', 0)
                        uav.set('EK2_GPS_CHECK',    0)

                        settings = uav.get_settings()
                        keys = settings.keys()
                        keys.sort()
                        for key in keys:
                            self.log('UAV: %s\t\t%s' % (key, settings[key]))
                        self.log('')

                        break
                self.state = 1


            line = self.ser.readline()
            if line:
                line = line.strip()
            if not line:
                continue


            try:
                enable, offs, step, arm, trigger, abort = map(int, line.split(','))
            except ValueError:
                continue
            arm, trigger, abort = bool(arm), bool(trigger), bool(abort)
            #print 'enable', enable, 'offs', offs, 'step', step, 'arm', arm, 'trigger', trigger, 'abort', abort


            if not enable:
                self.m('Locked.\nPlease turn key.')
                continue


            if step:
                self.state = ((self.state + 1) % 6)
                if not self.state:
                    self.state += 1
                self.log('New state', self.state)


            if self.state == 1:
                if arm:
                    self.m('Please disarm.\nThen set latitude.')

                elif offs:
                    #self.log('Set latitude.')
                    self.ser.write("1,0,0,0\r")
                    offs /= 1000000.0
                    self.pos.lat += offs

                else:
                    self.m('Set latitude.')


            if self.state == 2:
                if arm:
                    self.m('Please disarm.\nThen set longitude.')

                elif offs:
                    self.ser.write("1,0,0,0\r")
                    offs /= 1000000.0
                    self.pos.lon += offs

                else:
                    self.m('Set longitude.')


            if self.state == 3:
                if not arm:
                    self.m('Please arm.\nThen transmit target.')

                elif trigger:
                    self.m('Transmitting target...')
                    time.sleep(1)
                    if not uav.set_target(self.pos):
                        self.m('Failed to transmit target.')
                    else:
                        self.m('Target transmitted.')

                else:
                    self.m('Transmit target.')


            # Start motors
            if self.state == 4:
                if not uav.get_target():
                    self.m('Before starting motors%s\ntransmit target.' % ('\narm and' if not arm else ''))

                elif not arm:
                    self.m('Please arm.\nThen start motors.')

                elif trigger:
                    self.m('Starting motors...')
                    time.sleep(1)
                    if uav.arm():
                        self.m('Motors running.')
                    else:
                        self.m('Failed to start motors.')

                else:
                    self.m('Start motors.')


            # Launch
            if self.state == 5:
                if not uav.get_target():
                    self.m('Before launch%s\ntransmit target\nand start motors.' % ('\narm and' if not arm else ''))

                elif not uav.is_armed():
                    self.m('Before launch start motors.')

                elif not arm:
                    self.m('Please arm.\nThen launch.')

                elif trigger:
                    self.m('Launching...')
                    if uav.launch():
                        self.m('Launched')
                    else:
                        self.m('Failed to launch.')

                else:
                    self.m('Launch.')


            # Abort
            elif abort:
                if not arm:
                    self.m('Please arm.\nThen abort.')
                else:
                    uav.land()
                    uav.disarm()




    def get_position(self):
        """Get calculated GPS position or None
        relative to absolute local GPS position.
        """
        return self.pos


board = Board()


