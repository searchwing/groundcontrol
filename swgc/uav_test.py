# -*- coding: utf-8 -*-
"""Test the vehicle module.
"""
import time

from . import vehicle
from . settings import *


def run():
    """Call to test.
    """
    vehicle.connect(UAV_ADDRESS)
    vehicle.wait_for_connection()
    vehicle.wait_for_position()
    if vehicle.test_copter_set_target() and vehicle.arm():
        vehicle.launch()

    while 1:
        time.sleep(1000)
