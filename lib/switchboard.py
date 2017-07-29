# -*- coding: utf-8 -*-
"""The switchboard.
"""
import time, traceback

import ui
from . gps import gps
from . uav import uav
from . settings import *
from . serialthread import SerialThread


STATE_NO_STATE     = 0
STATE_WAIT_FOR_POS = 1
STATE_WAIT_FOR_UAV = 2
STATE_SET_LAT      = 3
STATE_SET_LON      = 4
STATE_SET_TARGET   = 5
STATE_START_MOTORS = 6


def state2str(state):
    return (
        'STATE_NO_STATE',
        'STATE_WAIT_FOR_POS',
        'STATE_WAIT_FOR_UAV',
        'STATE_SET_LAT',
        'STATE_SET_LON',
        'STATE_SET_TARGET',
        'STATE_START_MOTORS',
    )[state]


LIGHT_1 = 1
LIGHT_2 = 2
LIGHT_3 = 3
LIGHT_4 = 4

LIGHT_OFF   = 0
LIGHT_ON    = 1
LIGHT_BLINK = 2




class Board(SerialThread):
    """Async (re)connect local serial switchboard,
     control UI and UAV.
    """

    def __init__(self, *args, **kwargs):
        super(Board, self).__init__('Board', BOARD_PORT, BOARD_BAUD)
        self.msg, self.pos = None, None
        self.state = STATE_NO_STATE


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
                '1,1,1,1',
                '0,0,0,0'):
            self.ser.write('%s\r' % code)
            time.sleep(0.2)


    def light_trigger(self, light, mode):
        self.ser.write('0,0,%i,0\r' % mode


    def m(self, msg):
        if not msg == self.msg:
            self.msg = msg
            self.log(self.msg)


    def get_message(self):
        #return '(%s)\n%s' % (state2str(self.state), self.msg or '')
        return self.msg or ''


    def goto_state(self, state):
        self.state = state
        self.log('Board: new state %s' % state2str(self.state))


    def work(self):
        while 1:
            try:
                self._work()
            except Exception, e:
                print e
                traceback.print_exc()


    def _work(self):
        self.iluminate()


        if not self.pos:
            self.goto_state(STATE_WAIT_FOR_POS)
            self.m('Waiting for local position...')
            ui.notify()

            while 1:
                time.sleep(1)
                self.pos = gps.get_position()
                if self.pos:
                    break
            self.m('...found local position')
            ui.notify()
            time.sleep(1)
            self.goto_state(STATE_SET_LAT)


        while 1:
            ui.notify()


            if not uav.get_uav():
                next_state = self.state
                self.goto_state(STATE_WAIT_FOR_UAV)
                self.m('Waiting for UAV...')

                while 1:
                    time.sleep(1)
                    if uav.get_uav():
                        break
                self.m('...found UAV')
                ui.notify()
                time.slee(1)
                self.goto_state(next_state)

                uav.set('ARMING_CHECK',     9) # ?
                uav.set('BRD_SAFETYENABLE', 0) # ?
                uav.set('EK2_GPS_CHECK',    0) # ?

                settings = uav.get_settings()
                keys = settings.keys()
                keys.sort()
                for key in keys:
                    self.log('UAV: %s\t\t%s' % (key, settings[key]))
                self.log('')




            line = self.ser.readline()
            if line:
                line = line.strip()
            if not line:
                continue
            try:
                unlock, offs, step, arm, trigger, abort = map(int, line.split(','))
            except ValueError:
                continue
            arm, trigger, abort = bool(arm), bool(trigger), bool(abort)




            if not unlock:
                self.m('Locked.\nPlease turn key.')
                continue

            if abort:
                if not arm:
                    self.m('Please arm.\nThen abort.')
                    continue

                uav.land()
                uav.disarm()
                self.goto_state(STATE_NO_STATE)
                break




            if self.state == STATE_SET_LAT:

                if arm:
                    self.m('Please disarm.\nThen set latitude.')
                else:
                    self.m('Set latitude.')

                    if step:
                        step = None
                        self.goto_state(STATE_SET_LON)

                    elif offs:
                        self.ser.write("1,0,0,0\r")
                        offs /= 1000000.0
                        self.pos.lat += offs


            if self.state == STATE_SET_LON:
                if arm:
                    self.m('Please disarm.\nThen set longitude.')
                else:
                    self.m('Set longitude.')

                    if step:
                        step = None
                        self.goto_state(STATE_SET_TARGET)

                    elif offs:
                        self.ser.write("1,0,0,0\r")
                        offs /= 1000000.0
                        self.pos.lon += offs




            if self.state == STATE_SET_TARGET:
                if not arm:
                    self.m('Please arm.\nThen transmit target.')

                elif trigger:
                    trigger = None
                    self.m('Transmitting target...')
                    time.sleep(1)

                    if uav.set_target(self.pos):
                        self.m('Target transmitted.')
                        self.goto_state('STATE_START_MOTORS')
                    else:
                        self.m('Failed to transmit target.')

                else:
                    self.m('Transmit target.')


            if self.state == STATE_START_MOTORS:
                if not arm:
                    self.m('Please arm.\nThen start motors.')

                elif trigger:
                    trigger = None
                    self.m('Starting motors...')
                    time.sleep(1)

                    if uav.arm():
                        self.m('Motors running.')
                        self.goto_state('STATE_LAUNCH')
                    else:
                        self.m('Failed to start motors.')

                else:
                    self.m('Start motors.')


            if self.state == STATE_LAUNCH:
                if not arm:
                    self.m('Please arm.\nThen launch.')

                elif trigger:
                    trigger = None
                    self.m('Launching...')
                    time.sleep(1)

                    if uav.launch():
                        self.m('Launched')
                        self.goto_state(STATE_NO_STATE)
                        return

                    else:
                        self.m('Failed to launch.')

                else:
                    self.m('Launch.')





    def get_position(self):
        """Get calculated GPS position or None
        relative to absolute local GPS position.
        """
        return self.pos


board = Board()

