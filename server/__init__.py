# -*- coding: utf-8 -*-
"""General initializations.
Start flask server.
"""
import flask as _flask

from . uav import start as _uav_start
from . gps import start as _gps_start

from . uav import get                as get_uav
from . uav import get_states         as get_uav_states
from . gps import get_local_position as get_local_gps_position




# Allow us to reuse sockets after the are bound.
# http://stackoverflow.com/questions/25535975/release-python-flask-port-when-script-is-terminated
def _fix_socket():
    import socket
    socket.socket._bind = socket.socket.bind
    def socket_bind(self, *args, **kwargs):
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return socket.socket._bind(self, *args, **kwargs)
    socket.socket.bind = socket_bind
_fix_socket()




# the http server framework
app = _flask.Flask('uav')

# connecting uav
_uav_start()

# connecting gps
_gps_start()

