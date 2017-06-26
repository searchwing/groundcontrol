# -*- coding: utf-8 -*-
"""(Re)connect UAV in threaded loop
"""
import socket, time, threading, exceptions

import dronekit, dronekit_sitl

from . geo import Position
from . settings import *




class UAV(threading.Thread):
    """(Re)connect UAV in threaded loop
    Internal only.
    """

    def __init__(self, *args, **kwargs):
        super(self.__class__, self).__init__(*args, **kwargs)
        self.daemon = True

        self.last_heartbeat = 0
        self.uav = None
        self.states = {}


    def run(self):
        if SITL:
            sitl = dronekit_sitl.start_default() # it's a copter
            connection_string = sitl.connection_string()
            print '!!! Simulation. Start SITL copter !!!', connection_string
        else:
            connection_string = UAV_ADDRESS

        while 1:
            if self.uav and UAV_TIMEOUT_CONNECTION and \
                    time.time() - self.last_heartbeat > UAV_TIMEOUT_CONNECTION:
                print 'Lost connection to UAV'
                self.uav.close()
                self.uav = None
                self.states = {}

            if not self.uav:
                print 'Connecting UAV %s' % connection_string
                uav = None
                try:
                    uav = dronekit.connect(
                            connection_string,
                            baud              = UAV_BAUD,
                            rate              = UAV_REFRESH_RATE,
                            heartbeat_timeout = UAV_TIMEOUT_HEARTBEAT,
                            wait_ready        = True)

                except socket.error:
                    print 'Connecting UAV error: no server'

                except exceptions.OSError:
                    print 'Connecting UAV error: no serial'

                except dronekit.APIException:
                    print 'Connecting UAV error: timeout'

                except Exception, e:
                    print 'Connecting UAV error: unkown', e

                if not uav:
                    print 'Connecting UAV failed'

                else:
                    print 'UAV connected'
                    self.uav = uav

                    self.last_heartbeat = time.time()
                    self.uav.add_attribute_listener(
                            'last_heartbeat', self.on_heartbeat)
                    self.uav.add_attribute_listener(
                            '*', self.on_any)

            if self.uav and not UAV_TIMEOUT_CONNECTION:
                break

            time.sleep(UAV_TIMEOUT_CONNECTION_TEST)


    def on_heartbeat(self, *args, **kwargs):
        self.last_heartbeat = time.time()


    def on_any(self, uav, name, value):
        self.states[name] = value




    def get_states(self):
        return _vehicle2states(self.uav)


    def ts(self):
        pass


    def get_position(self):
        states = self.get_states()
        if states:
            return Position(
                lat = states.get('location_global_frame_lat'),
                lon = states.get('location_global_frame_lon'),
                alt = states.get('location_global_frame_alt'))
        return None


    def prearm(self):
        """UAV prearm check.
        """
        vehicle = self.uav
        if not vehicle:
            print 'UAV not yet connected'
            return False

        cnt = 0
        while not vehicle.is_armable:
            print 'Waiting until UAV is armable...'
            cnt += 1
            if cnt > UAV_TIMEOUT_PREARM:
                print 'UAV timeout armable'
                return False
            time.sleep(1)
        print 'UAV is armable'

        cnt = 0
        while vehicle.gps_0.fix_type < 2:
            print 'Waiting for UAV GPS...'
            cnt += 1
            if cnt > TIMEOUT_GPS:
                print 'UAV timeout gps'
                return False

        print 'UAV GPS fix:', vehicle.gps_0.fix_type
        return True


    def arm(self):
        vehicle = self.uav
        if not vehicle:
            print 'UAV not yet connected'
            return False

        if not self.prearm(): # prearm if not yet done
            return

        vehicle.mode = dronekit.VehicleMode('GUIDED') # ?

        vehicle.armed = True
        cnt = 0
        while not vehicle.armed:
            print 'Waiting until UAV is armed...'
            if cnt > UAV_TIMEOUT_ARM:
                print 'UAV timeout arm'
                return False
            time.sleep(1)

        print 'UAV is armed'
        return True


uav = UAV()


def _vehicle2states(vehicle):
    if not vehicle:
        return {}
    return {
        'version_major'             : vehicle.version.major,
        'version_patch'             : vehicle.version.patch,
        'version_release'           : vehicle.version.release,
        'system_status'             : vehicle.system_status.state,
        'is_armable'                : vehicle.is_armable,
        'armed'                     : vehicle.armed,
        'gps_fix'                   : vehicle.gps_0.fix_type,
        'gps_sats'                  : vehicle.gps_0.satellites_visible,
        'location_global_frame_alt' : vehicle.location.global_frame.alt,
        'location_global_frame_lon' : vehicle.location.global_frame.lon,
        'location_global_frame_lat' : vehicle.location.global_frame.lat,
        'rangefinder_distance'      : vehicle.rangefinder.distance,
        'rangefinder_voltage'       : vehicle.rangefinder.voltage,
        'attitude_pitch'            : vehicle.attitude.pitch,
        'attitude_roll'             : vehicle.attitude.roll,
        'attitude_yaw'              : vehicle.attitude.yaw,
        'velocity'                  : vehicle.velocity,
        'airspeed'                  : vehicle.airspeed,
        'groundspeed'               : vehicle.groundspeed,
        'heading'                   : vehicle.heading,
        'battery_voltage'           : vehicle.battery.voltage,
        'battery_current'           : vehicle.battery.current,
        'battery_level'             : vehicle.battery.current,
        'last_heartbeat'            : vehicle.last_heartbeat,
    }
