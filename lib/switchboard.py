# -*- coding: utf-8 -*-
"""The switchboard.
"""
import time, traceback

import ui
import framebuffer as fb
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
STATE_LAUNCH       = 7


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


LIGHT_ONE   = 0
LIGHT_TWO   = 1
LIGHT_THREE = 2
LIGHT_FOUR  = 3

LIGHT_OFF   = '0'
LIGHT_ON    = '1'
LIGHT_BLINK = '2'




class Board(SerialThread):
    """Async (re)connect local serial switchboard,
     control UI and UAV.
    """

    def __init__(self, *args, **kwargs):
        super(Board, self).__init__('Board', BOARD_PORT, BOARD_BAUD)
        self.state = STATE_NO_STATE
        self.msg, self.pos = None, None
        self.lightsList = [LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF,]


    def ligthsInit(self):
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


    def lights(self):
        self.ser.write('%s\r' % ','.join(self.lightsList))

    def light(self, idx, mode, flush = True):
        self.lightsList[idx] = '%s' % mode
        if flush:
            self.lights()

    def lightsOn(self, flush = True):
        self.lightsList = [LIGHT_ON, LIGHT_ON, LIGHT_ON, LIGHT_ON,]
        if flush:
            self.lights()

    def lightsOff(self, flush = True):
        self.lightsList = [LIGHT_OFF, LIGHT_OFF, LIGHT_OFF, LIGHT_OFF,]
        if flush:
            self.lights()

    def lightsBlink(self, flush = True):
        self.lightsList = [LIGHT_BLINK, LIGHT_BLINK, LIGHT_BLINK, LIGHT_BLINK,]
        if flush:
            self.lights()

    def lightsSignal(self):
        self.lightsOn()
        time.sleep(1)
        self.lightsOff()




    def lightsTest(self):
        self.lightsOff()
        time.sleep(1)
        self.light(LIGHT_ONE,   LIGHT_ON)
        time.sleep(1)
        self.lightsOff()
        time.sleep(1)
        self.light(LIGHT_TWO,   LIGHT_ON)
        time.sleep(1)
        self.lightsOff()
        time.sleep(1)
        self.light(LIGHT_THREE, LIGHT_ON)
        time.sleep(1)
        self.lightsOff()
        time.sleep(1)
        self.light(LIGHT_FOUR,  LIGHT_ON)
        time.sleep(1)
        self.lightsOff()
        time.sleep(1)


    def m(self, msg):
        if not msg == self.msg:
            self.msg = msg
            self.log(self.msg)
            ui.notify()

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
                time.sleep(2)




    def _work(self):
        ui.clear()
        self.ligthsInit()

        self.goto_state(STATE_NO_STATE)


        self.pos = gps.get_position()
        if not self.pos:
            self.goto_state(STATE_WAIT_FOR_POS)
            self.m('Waiting for local position...')
            self.lightsSignal()

            while 1:
                self.pos = gps.get_position()
                if self.pos:
                    break
                time.sleep(1)
            self.lightsSignal()

        self.m('...Found local position')
        time.sleep(2)




        loop_started = True
        while 1:
            ui.notify()

            
            if not uav.get_uav():
                self.goto_state(STATE_WAIT_FOR_UAV)
                self.m('Waiting for UAV...')
                while 1:
                    if uav.get_uav():
                        break
                    time.sleep(0.5)

                #uav.set('ARMING_CHECK',     9) # ?
                #uav.set('BRD_SAFETYENABLE', 0) # ?
                #uav.set('EK2_GPS_CHECK',    0) # ?

                settings = uav.get_settings()
                keys = settings.keys()
                keys.sort()
                for key in keys:
                    self.log('UAV: %s\t\t%s' % (key, settings[key]))
                self.log('')

            if loop_started:
                loop_started = False
                self.goto_state(STATE_SET_LAT)
                self.m('...Found UAV')
                self.lightsOn()
                time.sleep(2)
                self.lightsOff()
                self.m('Now start flight preparations.')
                time.sleep(2)




            line = self.ser.readline()
            if line:
                line = line.strip()
            if not line:
                continue
            try:
                unlock, offs, left, arm, right, abort = map(int, line.split(','))
            except ValueError:
                continue
            arm, right, abort = bool(arm), bool(right), bool(abort)




            if not unlock:
                self.m('Locked.\nPlease turn key.')
                self.lightsOff()
                continue


            if abort:
                if not arm:
                    self.m('Please arm.\nThen abort.')
                    self.lightsOff(False)
                    self.light(LIGHT_TWO, LIGHT_ON)
                    continue

                self.m('Aborted')
                self.lightsOff(False)
                self.light(LIGHT_FOUR, LIGHT_ON)
                self.goto_state(STATE_NO_STATE)
                uav.land()
                uav.disarm()
                time.sleep(5)
                self.lightsOff()
                return




            if self.state == STATE_SET_LAT:
                self.light(LIGHT_ONE, LIGHT_ON)

                if arm:
                    self.m('Please disarm.\nThen set latitude.')
                    self.light(LIGHT_TWO, LIGHT_ON)
                else:
                    self.m('Set latitude.')
                    self.light(LIGHT_TWO, LIGHT_OFF)

                    if left:
                        left = None
                        self.lightsOff(False)
                        self.goto_state(STATE_SET_LON)
                    elif offs:
                        self.ser.write("1,0,0,0\r")
                        offs /= 1000000.0
                        self.pos.lat += offs




            if self.state == STATE_SET_LON:
                self.light(LIGHT_ONE, LIGHT_ON)

                if arm:
                    self.m('Please disarm.\nThen set longitude.')
                    self.lightsOff(False)
                    self.light(LIGHT_TWO, LIGHT_ON)
                else:
                    self.m('Set longitude.')
                    self.light(LIGHT_TWO, LIGHT_OFF)

                    if left:
                        left = None
                        self.lightsOff(False)
                        self.goto_state(STATE_SET_TARGET)
                    elif offs:
                        self.ser.write("1,0,0,0\r")
                        offs /= 1000000.0
                        self.pos.lon += offs




            if self.state == STATE_SET_TARGET:
                self.light(LIGHT_THREE, LIGHT_ON)

                if not arm:
                    self.m('Please arm.\nThen transmit target.')
                    self.light(LIGHT_TWO, LIGHT_ON)
                else:
                    self.light(LIGHT_TWO, LIGHT_OFF)

                    if right:
                        right = None
                        self.m('Transmitting target...')
                        time.sleep(1)

                        if uav.set_target(self.pos):
                            self.m('Target transmitted.')
                            self.lightsOff(False)
                            self.goto_state(STATE_START_MOTORS)
                        else:
                            self.m('Failed to transmit target.')
                    else:
                        self.m('Transmit target.')




            if self.state == STATE_START_MOTORS:
                self.light(LIGHT_THREE, LIGHT_ON)

                if not arm:
                    self.m('Please arm.\nThen start motors.')
                    self.light(LIGHT_TWO, LIGHT_ON)
                else:
                    self.light(LIGHT_TWO, LIGHT_OFF)

                    if right:
                        right = None
                        self.m('Starting motors...')
                        time.sleep(1)

                        if uav.arm():
                            self.m('Motors running.')
                            self.lightsOff(False)
                            self.goto_state(STATE_LAUNCH)
                        else:
                            self.m('Failed to start motors.')
                    else:
                        self.m('Start motors.')


            if self.state == STATE_LAUNCH:
                self.light(LIGHT_THREE, LIGHT_ON)

                if not arm:
                    self.m('Please arm.\nThen launch.')
                    self.light(LIGHT_TWO, LIGHT_ON)
                else:
                    self.light(LIGHT_TWO, LIGHT_OFF)

                    if right:
                        right = None
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

