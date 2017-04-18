# -*- coding: utf-8 -*-
"""Server app init.
Init flask
(Re)connect UAV in threaded loop
@author sascha@searchwing.org
"""
import socket, time, threading, exceptions
from datetime import datetime

import flask, dronekit




# Allow us to reuse sockets after the are bound.
# http://stackoverflow.com/questions/25535975/release-python-flask-port-when-script-is-terminated
def _fix_socket():
    socket.socket._bind = socket.socket.bind
    def socket_bind(self, *args, **kwargs):
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return socket.socket._bind(self, *args, **kwargs)
    socket.socket.bind = socket_bind
_fix_socket()




# the http server framework
app = flask.Flask('searchwing groundcontrol')




UAV_ADDRESS  = 'tcp:127.0.0.1:5760'
UAV_HEARTBEAT_TIMEOUT       = 20 # seconds
UAV_CONNECTION_TIMEOUT      =  5 # seconds
UAV_CONNECTION_TEST_TIMEOUT =  5 # seconds


# (Re)connect uav in threaded loop
class _UAV(threading.Thread):
    """Internal only.
    """

    def __init__(self, *args, **kwargs):
        super(self.__class__, self).__init__(*args, **kwargs)
        self.daemon = True

        self.uav = None
        self.last_heartbeat = 0

        self.states = {}

    def run(self):
        while 1:
            if time.time() - self.last_heartbeat > UAV_CONNECTION_TIMEOUT:
                if self.uav:
                    print 'Connection lost UAV %s' % UAV_ADDRESS
                    self.uav.close()
                    self.uav = None

            if not self.uav:
                try:
                    print 'Connecting UAV %s' % UAV_ADDRESS
                    self.uav = dronekit.connect(
                            UAV_ADDRESS,
                            wait_ready = True,
                            heartbeat_timeout = UAV_HEARTBEAT_TIMEOUT)

                except socket.error:
                    print 'error: no server'

                except exceptions.OSError:
                    print 'error: no serial'

                except dronekit.APIException:
                    print 'error: timeout'

                except Exception, e:
                    print 'error: unkown', e

                if not self.uav:
                    print 'Connecting failed UAV %s' % UAV_ADDRESS

                else:
                    print 'Connected UAV %s' % UAV_ADDRESS
                    self.last_heartbeat = time.time()
                    self.uav.add_attribute_listener(
                            'last_heartbeat', self.on_heartbeat)

                    self.uav.add_attribute_listener(
                            '*', self.on_any)

            time.sleep(UAV_CONNECTION_TEST_TIMEOUT)


    def on_heartbeat(self, *args, **kwargs):
        self.last_heartbeat = time.time()

    def on_any(self, uav, name, value):
        self.states[name] = value


_uav = _UAV()
_uav.start()

def get_uav():
    """Get the connected UAV or None.
    """
    return _uav.uav

