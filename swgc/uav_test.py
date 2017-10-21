# -*- coding: utf-8 -*-
"""Test the vehicle module.
"""
import time

from . import vehicle
from . settings import UAV_ADDRESS


def run():
    """Call to test.
    """
    try:
        if not vehicle.connect(UAV_ADDRESS):
            return
        if not vehicle.wait_for_position():
            return

        pos = vehicle.get_position()
        pos = pos.get_location_by_offset_meters_and_heading(10, 0)
        if not vehicle.set_target(pos):
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
