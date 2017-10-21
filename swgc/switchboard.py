# -*- coding: utf-8 -*-
"""The switchboard.
"""
import time

from . import sync
from . settings import ALTITUDE
from . serialthread import SerialThread


STATE_NO_STATE              = 0
STATE_WAIT_FOR_POS          = 1
STATE_WAIT_FOR_UAV          = 2
STATE_WAIT_FOR_UAV_POSITION = 3
STATE_SET_LAT               = 4
STATE_SET_LON               = 5
STATE_SET_TARGET            = 6
STATE_START_MOTORS          = 7
STATE_LAUNCH                = 8
STATE_FLYING                = 9


def state2str(state):
    return (
        'STATE_NO_STATE',
        'STATE_WAIT_FOR_POS',
        'STATE_WAIT_FOR_UAV',
        'STATE_WAIT_FOR_UAV_POSITION',
        'STATE_SET_LAT',
        'STATE_SET_LON',
        'STATE_SET_TARGET',
        'STATE_START_MOTORS',
        'STATE_LAUNCH',
        'STATE_FLYING',
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

    def __init__(self, name, port, baud, gps, uav):
        super(Board, self).__init__(name = name, port = port, baud = baud)
        self.gps, self.uav = gps, uav
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
            sync.notify()


    def get_message(self):
        return self.msg or ''


    def goto_state(self, state):
        self.state = state
        self.log('new state %s' % state2str(self.state))


    def get_position(self):
        """Get calculated GPS position or None
        relative to absolute local GPS position.
        """
        return self.pos




    def work(self):
        self.ligthsInit()

        self.goto_state(STATE_NO_STATE)


        self.pos = self.gps.get_position()
        if not self.pos:
            self.goto_state(STATE_WAIT_FOR_POS)
            self.m('Waiting for local position...')
            self.lightsSignal()
            while 1:
                sync.wait(1)
                self.pos = self.gps.get_position()
                if self.pos:
                    break
            self.m('...Found local position')
            self.lightsSignal()
            time.sleep(1)


        if not self.uav.is_connected():
            self.goto_state(STATE_WAIT_FOR_UAV)
            self.m('Waiting for UAV...')
            while 1:
                sync.wait(1)
                if self.uav.is_connected():
                    break
            self.m('...Found UAV')
            time.sleep(1)


        if not self.uav.get_position():
            self.goto_state(STATE_WAIT_FOR_UAV_POSITION)
            self.m('Waiting for UAV Position...')
            while 1:
                sync.wait(1)
                if self.uav.get_position():
                    break
            self.m('...Found UAV Position')
            time.sleep(1)


        loop_started = True
        while 1:
            sync.notify()


            if loop_started:
                loop_started = False
                self.goto_state(STATE_SET_LAT)
                self.lightsOn()
                time.sleep(2)
                self.lightsOff()
                self.m('Now start flight preparations.')
                time.sleep(1)


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

                if self.uav.is_waiting():
                    self.m('Abort')
                    self.lightsOff(False)
                    self.light(LIGHT_FOUR, LIGHT_ON)
                    self.goto_state(STATE_NO_STATE)
                    self.uav.reset()
                    time.sleep(5)
                    self.lightsOff()
                    return

                elif self.uav.is_flying():
                    self.m('Return to Land')
                    self.uav.return_to_land()


            if self.uav.is_flying() or self.uav.is_rtl():
                self.m('Still in the air')
                continue




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

                        self.pos.alt = ALTITUDE
                        if self.uav.set_target(self.pos):
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

                        if self.uav.arm():
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

                        if self.uav.launch():
                            self.m('Launched')
                            self.goto_state(STATE_FLYING)
                            return

                        else:
                            self.m('Failed to launch.')

                    else:
                        self.m('Launch.')
