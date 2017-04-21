# -*- coding: utf-8 -*-
"""General initializations.
Start flask server.
@author sascha@searchwing.org
"""
import socket

import flask

from . import uav as _uav
from . import gps as _gps




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
app = flask.Flask('uav')




_uav.start()

def get_uav():
    """Get connected UAV or None.
    """
    return _uav.get_uav()




_gps.start()

def get_position():
    """Get latest known position or None.
    """
    return _gps.get_position()

