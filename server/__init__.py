# -*- coding: utf-8 -*-
"""Server app init.
Init flask
(Re)connect UAV in threaded loop
@author sascha@searchwing.org
"""
import socket, time, threading
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




UAV_ADDRESS           = 'tcp:127.0.0.1:5760'
UAV_RECONNECT_TIMEOUT = 2 # secons
UAV_CONNECTED_TIMEOUT = 5 # seconds


# (Re)connect uav in threaded loop
class _UAV(threading.Thread):
    """Internal only.
    """

    def __init__(self, *args, **kwargs):
        super(self.__class__, self).__init__(*args, **kwargs)
        self.uav = None
        self.last_heartbeat = 0

        self.daemon = True
        self.start()


    def run(self):
        while 1:
            if time.time() - self.last_heartbeat > UAV_CONNECTED_TIMEOUT:
                if self.uav:
                    print 'Connection lost UAV %s' % UAV_ADDRESS
                self.uav = None

            if not self.uav:
                print 'Connecting UAV %s' % UAV_ADDRESS
                try:
                    self.uav = dronekit.connect(
                            UAV_ADDRESS, wait_ready = True, rate = 10)
                except Exception, e:
                    print 'Connecting failed UAV %s' % UAV_ADDRESS

                if self.uav:
                    print 'Connected UAV %s' % UAV_ADDRESS
                    last_heartbeat = time.time()
                    self.uav.add_attribute_listener(
                            'last_heartbeat', self.on_heartbeat)

            time.sleep(UAV_RECONNECT_TIMEOUT)


    def on_heartbeat(self, *args, **kwargs):
        self.last_heartbeat = time.time()


_uav = _UAV()

def get_uav():
    """Get the connected UAV or None.
    """
    return _uav.uav

