# -*- coding: utf-8 -*-
"""The UAV this app controls.
Object wrapper for the actual vehicle module.
"""
import time, threading

from . import vehicle


class UAV(threading.Thread):
    """Singelton object for the vehicle module.
    Keeps connecting the vehicle, delegates all calls to it.
    """
    def __init__(self, address):
        super(UAV, self).__init__()
        self.daemon = True
        self.address = address
        self.running, self.closed = False, True


    def start(self):
        self.running, self.closed = True, False
        super(UAV, self).start()


    def close(self):
        """Not blocking.
        """
        self.running = False


    def is_closed(self):
        return not vehicle.is_connected() or self.closed


    def run(self):
        while self.running:
            try:
                if not vehicle.is_connected():
                    vehicle.connect(self.address)

                if vehicle.is_flying() or vehicle.is_rtl():
                    print
                    vehicle.log_state()
                    time.sleep(1)

            except BaseException, e:
                print e
                time.sleep(1)

        if vehicle.is_connected():
            try:
                vehicle.close()
            except BaseException, e:
                print e

        self.closed = True


    def __getattr__(self, name):
        """Delegate all calls to the wrapped vehicle module.
        """
        return getattr(vehicle, name)
