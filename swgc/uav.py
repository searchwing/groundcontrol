# -*- coding: utf-8 -*-
"""The UAV this app controls.
Object wrapper for actual vehicle module.
"""
import threading, traceback

from . import vehicle, sync


class UAV(threading.Thread):
    """Singelton object for the vehicle module.
    Keeps connecting the vehicle, delegates all calls to it.
    """
    def __init__(self, address):
        super(UAV, self).__init__()
        self.address = address
        self.daemon = True


    def run(self):
        while 1:
            try:
                if not vehicle.is_connected():
                    vehicle.connect(self.address)

                if vehicle.is_flying():
                    vehicle.log_state()

            except BaseException, e:
                # Continue at any price
                print e
                traceback.print_exc()

            sync.wait(1)

    def __getattr__(self, name):
        """Delegate all calls to the wrapped vehicle module.
        """
        return getattr(vehicle, name)




# a global singleton
_uav = None

def start(address):
    """Instanciate and start a global singleton.
    """
    global _uav
    _uav = UAV(address)
    _uav.start()

class _UAV():
    def __getattr__(self, attr):
        return getattr(_uav, attr)

uav = _UAV()
