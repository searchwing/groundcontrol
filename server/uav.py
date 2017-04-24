# -*- coding: utf-8 -*-
"""(Re)connect UAV in threaded loop
"""
import socket, time, threading, exceptions
import dronekit
from settings import *




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
                print 'UAV Connection lost'
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
                    print 'error: unkown', e

                if not self.uav:
                    print 'Connecting UAV failed'

                else:
                    print 'Connecting UAV ok'
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




_uav = _UAV()


def get():
    """Get connected UAV or None.
    """
    return _uav.uav


def start():
    """Start polling/connecting UAV.
    """
    _uav.start()


def get_states():
    """Get current UAV states.
    """
    return _uav.states

