# -*- coding: utf-8 -*-
"""(Re)connect UAV in threaded loop
"""
import socket, time, threading, exceptions

from pymavlink import mavutil
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
        self.target = None


    def log(self, *msg):
        msg = ' '.join((str(m) for m in msg))
        print 'UAV: %s' % msg


    def run(self):
        if SITL:
            sitl = dronekit_sitl.start_default() # it's a copter
            connection_string = sitl.connection_string()
            self.log('!!! Simulation. Start SITL copter !!!', connection_string)
        else:
            connection_string = UAV_ADDRESS

        while 1:
            if self.uav and UAV_TIMEOUT_CONNECTION and \
                    time.time() - self.last_heartbeat > UAV_TIMEOUT_CONNECTION:
                self.log('Lost connection to UAV')
                self.uav.close()
                self.uav = None
                self.states = {}

            if not self.uav:
                self.log('Connecting UAV %s' % connection_string)
                uav = None
                try:
                    uav = dronekit.connect(
                            connection_string,
                            baud              = UAV_BAUD,
                            rate              = UAV_REFRESH_RATE,
                            heartbeat_timeout = UAV_TIMEOUT_HEARTBEAT,
                            wait_ready        = True)

                except socket.error:
                    self.log('Connecting UAV error: no server')

                except exceptions.OSError:
                    self.log('Connecting UAV error: no serial')

                except dronekit.APIException:
                    self.log('Connecting UAV error: timeout')

                except Exception, e:
                    self.log('Connecting UAV error: unkown', e)

                if not uav:
                    self.log('Connecting UAV failed')

                else:
                    self.log('UAV connected')
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




    def get_uav(self):
        if not self.uav:
            self.log('Not connected')
        return self.uav


    def get_states(self):
        return _vehicle2states(self.uav)


    def get_position(self):
        states = self.get_states()
        if states:
            return Position(
                lat = states.get('location_global_frame_lat'),
                lon = states.get('location_global_frame_lon'),
                alt = states.get('location_global_frame_alt'))
        return None


    def set_target(self, pos):
        self.log('Set target', pos)
        uav = self.get_uav()
        if not uav: return

        self.target = pos

        uav.commands.clear()

        cur = self.get_position()
        cmd = dronekit.Command(0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 1, 0, 0, 0, 0, cur.lat, cur.lon, cur.alt)
        uav.commands.add(cmd)

        cmd = dronekit.Command(0, 0, 0,
           mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
           0, 0, 0, 0, 0, 0,
           pos.lat, pos.lon, pos.alt)
        uav.commands.add(cmd)

        uav.commands.upload()


    def prearm(self):
        """UAV prearm check.
        """
        uav = self.get_uav()
        if not uav: return False

        vehicle.mode = dronekit.VehicleMode('GUIDED')

        #cnt = 0
        #while not vehicle.is_armable:
        #    self.log('Waiting until UAV is armable...')
        #    cnt += 1
        #    if cnt > UAV_TIMEOUT_PREARM:
        #        self.log('timeout armable')
        #        return False
        #    time.sleep(1)
        #self.log('is armable')

        cnt = 0
        while vehicle.gps_0.fix_type < 2:
            self.log('Waiting for UAV GPS...')
            cnt += 1
            #if cnt > TIMEOUT_GPS:
            #    self.log('UAV timeout gps'
            #    return False

        self.log('GPS fix:', vehicle.gps_0.fix_type)
        return True


    def arm(self):
        self.log('Arm')
        vehicle = self.get_uav()
        if not vehicle: return False

        if not self.prearm(): # prearm if not yet done
            return

        vehicle.mode = dronekit.VehicleMode('GUIDED') # ?

        vehicle.armed = True
        cnt = 0
        while not vehicle.armed:
            self.log('Waiting until UAV is armed...')
            if cnt > UAV_TIMEOUT_ARM:
                self.log('UAV timeout arm')
                return False
            time.sleep(1)

        self.log('UAV is armed')
        return True


    def land(self):
        self.log('Land')
        vehicle = self.get_uav()
        if not vehicle: return False

        vehicle.mode = dronekit.VehicleMode('LAND')
        vehicle.flush()
        return True


    def disarm(self):
        self.log('Disarm')
        vehicle = self.get_uav()
        if not vehicle: return False

        vehicle.armed = False
        vehicle.flush()
        return True


    def launch(self):
        self.log('Launch')
        vehicle = self.get_uav()
        if not vehicle: return False

        uav.simple_takeoff(self.target.alt)
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
