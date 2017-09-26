# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
"""The UAV this app controls.
Object rapper for vehicle module.
"""
import threading, time

from . import settings, vehicle


class UAV(threading.Thread):

    def __init__(self, *args, **kwargs):
        threading.Thread.__init__(self, *args, **kwargs)
        self.daemon = True

    def run(self):
        while 1:
            if vehicle.is_connected():
                time.sleep(10)

            else:
                try:
                    if not vehicle.is_connected():
                        vehicle.connect(settings.UAV_ADDRESS)
                except Exception, e:
                    print e
                    traceback.print_exc()
                if not vehicle.is_connected():
                    time.sleep(1)

    def __getattr__(self, name):
        """Proxy all calls to vehicle module.
        """
        return getattr(vehicle, name)


uav = UAV()

