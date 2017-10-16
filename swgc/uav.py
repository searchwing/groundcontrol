# -*- coding: utf-8 -*-
"""The UAV this app controls.
Object wrapper for actual vehicle module.
"""
import time, threading

from . import vehicle


class UAV(threading.Thread):
    """Singelton object for the vehicle module.
    Keeps connecting the vehicle, delegates all calls to it.
    """
    def __init__(self):
        super(UAV, self).__init__()
        self.daemon = True
        self.address, self.running, self.closed = None, False, True


    def start(self, address):
        self.address, self.running, self.closed = address, True, False
        super(UAV, self).start()


    def close(self):
        self.running = False


    def is_closed(self):
        return not vehicle.is_connected() or self.closed


    def run(self):
        while self.running:
            try:
                if not vehicle.is_connected():
                    vehicle.connect(self.address)

                if vehicle.is_flying() or vehicle.is_rtl():
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


uav = UAV()
