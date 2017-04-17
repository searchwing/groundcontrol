import socket, time, threading
from datetime import datetime

from flask import Flask

import dronekit



# Allow us to reuse sockets after the are bound.
# http://stackoverflow.com/questions/25535975/release-python-flask-port-when-script-is-terminated
socket.socket._bind = socket.socket.bind
def socket_bind(self, *args, **kwargs):
    self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    return socket.socket._bind(self, *args, **kwargs)
socket.socket.bind = socket_bind


app = Flask('searchwing groundcontrol')


UAV_ADDRESS = 'tcp:127.0.0.1:5760'
UAV = None

def _connect_uav():
    global UAV
    while 1:
        if not UAV:
            try:
                print 'Connecting UAV on %s ...' % UAV_ADDRESS
                UAV = dronekit.connect(UAV_ADDRESS, wait_ready = True, rate = 10)
                print 'Connected UAV on %s ...' % UAV_ADDRESS
            except Exception, e:
                print e
        print '---------'
        time.sleep(2)
threading.Thread(target = _connect_uav).start()

def get_uav():
    return UAV

