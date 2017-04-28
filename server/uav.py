# -*- coding: utf-8 -*-
"""(Re)connect UAV in threaded loop
"""
import socket, time, threading, exceptions
import dronekit
from settings import *




TIMEOUT_PREARM = 10
TIMEOUT_ARM    = 10
TIMEOUT_GPS    = 10




class _UAV(threading.Thread):
    """(Re)connect UAV in threaded loop
    Internal only.
    """

    def __init__(self, *args, **kwargs):
        super(self.__class__, self).__init__(*args, **kwargs)
        self.daemon = True

        self.last_heartbeat = 0
        self.uav = None
        self.states = {}


    def run(self):
        while 1:
            if self.uav and UAV_CONNECTION_TIMEOUT and \
                    time.time() - self.last_heartbeat > UAV_CONNECTION_TIMEOUT:
                print 'Lost connection to UAV'
                self.uav.close()
                self.uav = None
                self.states = {}

            if not self.uav:
                print 'Connecting UAV %s' % UAV_ADDRESS
                try:
                    self.uav = dronekit.connect(
                            UAV_ADDRESS,
                            baud              = UAV_BAUD,
                            rate              = UAV_REFRESH_RATE,
                            wait_ready        = True,
                            heartbeat_timeout = UAV_HEARTBEAT_TIMEOUT)

                except socket.error:
                    print 'Connecting UAV error: no server'

                except exceptions.OSError:
                    print 'Connecting UAV error: no serial'

                except dronekit.APIException:
                    print 'Connecting UAV error: timeout'

                except Exception, e:
                    print 'Connecting UAV error: unkown', e

                if not self.uav:
                    print 'Connecting UAV failed'

                else:
                    print 'UAV connected'
                    self.last_heartbeat = time.time()
                    self.uav.add_attribute_listener(
                            'last_heartbeat', self.on_heartbeat)

                    self.uav.add_attribute_listener(
                            '*', self.on_any)

            if self.uav and not UAV_CONNECTION_TIMEOUT:
                break

            time.sleep(UAV_CONNECTION_TEST_TIMEOUT)


    def on_heartbeat(self, *args, **kwargs):
        self.last_heartbeat = time.time()


    def on_any(self, uav, name, value):
        self.states[name] = value




_uav = None
_uav_busy = False




def init():
    """Start polling/connecting UAV.
    Must be called on app initialization.
    """
    global _uav
    assert not _uav

    print 'UAV thread start'
    _uav = _UAV()
    _uav.start()


def get():
    """Get connected UAV or None.
    """
    assert _uav
    return _uav.uav


def get_states():
    """Get current UAV states.
    """
    assert _uav
    return _uav.states


def busy(is_busy):
    """Block vehicle for running commands.
    """
    assert _uav

    if is_busy:
        print 'Busy block UAV'
    else:
        print 'Busy unblock UAV'
    _uav_busy = is_busy


def is_ready():
    assert _uav

    if not _uav.uav:
        print 'No UAV conencted'
    elif _uav_busy:
        print 'UAV is busy'
    else:
        return True
    return False


def prearm():
    """UAV prearm check.
    """
    assert _uav

    if not is_ready():
        return False

    vehicle = get()
    assert vehicle

    cnt = 0
    while not vehicle.is_armable:
        print 'Waiting until UAV is armable...'
        if vehicle.mode.name == 'INITIALISING':
            print 'UAV is initializing...'
        cnt += 1
        if cnt > TIMEOUT_PREARM:
            print 'UAV timeout armable'
            return False
        time.sleep(1)
    print 'UAV is armable'

    cnt = 0
    while vehicle.gps_0.fix_type < 2:
        print 'Waiting for UAV GPS...'
        cnt += 1
        if cnt > TIMEOUT_GPS:
            print 'UAV timeout gps'
            return False

    print 'UAV GPS fix:', vehicle.gps_0.fix_type
    return True


def arm():
    assert _uav

    if not is_ready():
        return False

    vehicle = get()
    assert vehicle

    vehicle.armed = True
    cnt = 0
    while not vehicle.armed:
        print 'Waiting until UAV is armed...'
        if cnt > TIMEOUT_ARM:
            print 'UAV timeout arm'
            return False
        time.sleep(1)

    print 'UAV is armed'
    return True

