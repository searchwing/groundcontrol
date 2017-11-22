# -*- coding: utf-8 -*-
"""Test the vehicle module.
"""
import time

from . import vehicle
from . settings import UAV_ADDRESS


def run(connection_string = UAV_ADDRESS):
    """Call to test.
    """
    try:
        while not vehicle.connect(connection_string):
            time.sleep(1)

        vehicle.reset()

        if not vehicle.wait_for_position():
            return

        vehicle.set_home_position()

        pos = vehicle.get_home_position()
        pos = pos.get_location_by_offset_meters_and_heading(20, 0)
        pos.alt = 20

        if not vehicle.set_roi(pos):
            return

        if not vehicle.set_mission(pos):
            return

        if not vehicle.arm():
            return

        vehicle.launch()

        while 1:
            vehicle.log_state()
            time.sleep(1)

    except KeyboardInterrupt:
        print
        print 'Aboard'

    finally:
        vehicle.close()
