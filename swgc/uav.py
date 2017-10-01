# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
"""The UAV this app controls.
Object wrapper for actual vehicle module.
"""
import threading, traceback, time

from . import settings, vehicle, sync


class UAV(threading.Thread):
    """Singelton object for the vehicle module.
    Keeps connecting the vehicle, delegates all calls to it.
    """

    def run(self):
        while 1:
            try:
                if not vehicle.is_connected():
                    vehicle.connect(settings.UAV_ADDRESS)

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


# Singleton UAV, call start to get connected.
uav = UAV()
