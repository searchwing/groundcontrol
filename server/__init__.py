# -*- coding: utf-8 -*-
"""Initialize gps, uav.
Start flask server.
"""
import socket as _socket
import flask as _flask

import gps, uav




# Reuse flask sockets
# http://stackoverflow.com/questions/25535975/release-python-flask-port-when-script-is-terminated
def _fix_socket():
    _socket.socket._bind = _socket.socket.bind
    def socket_bind(self, *args, **kwargs):
        self.setsockopt(_socket.SOL_SOCKET, _socket.SO_REUSEADDR, 1)
        return _socket.socket._bind(self, *args, **kwargs)
    _socket.socket.bind = socket_bind
_fix_socket()




# connecting uav
uav.start()

# connecting gps
gps.start()

# the http server framework
app = _flask.Flask('uav')

