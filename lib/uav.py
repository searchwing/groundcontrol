# -*- coding: utf-8 -*-
"""(Re)connect UAV in threaded loop
"""
import socket, time, threading, exceptions

from pymavlink import mavutil
import dronekit, dronekit_sitl

from . geo import Position
from . settings import *


DISARMED  = 1
TARGETSET = 2
PREARMED  = 3
ARMED     = 4
LAUNCHED  = 5
LANDED    = 6
ABOART    = 7

INT2STATENAME = {
    1: 'DISARMED',
    2: 'TARGETSET',
    3: 'PREARMED',
    4: 'ARMED',
    5: 'LAUNCHED',
    6: 'LANDED',
    7: 'ABOART',
}


def state(state):
    """Returns decorator for UAV state dependend functions.
    Pass the state the function to be set by the function to decorate.
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            self = args[0]
            self.log('Going to state', INT2STATENAME[state])

            if not self.get_uav():
                return False

            if state == TARGETSET and not self.state == DISARMED:
                self.error('Please first disarm')
                return False

            if state == PREARMED and not self.state == TARGETSET:
                self.error('Please first set target')
                return False

            if state == ARMED and not self.state == PREARMED:
                self.error('Please first prearm')
                return False

            if state == LAUNCHED and not self.state == ARMED:
                self.error('Please first arm')
                return False

            if state == LANDED and not self.state == LAUNCHED:
                self.error('Not launched')
                return False

            if state == ABOART:
                pass

            ret = func(*args, **kwargs)
            if ret:
                self.state = state
            return ret

        return wrapper
    return decorator




class UAV(threading.Thread):
    """(Re)connect UAV in threaded loop
    """

    def __init__(self, *args, **kwargs):
        super(self.__class__, self).__init__(*args, **kwargs)
        self.daemon = True

        self.state = DISARMED

        self.last_heartbeat = 0
        self.uav            = None
        self.states         = {}
        self.target         = None


    def log(self, *msg):
        msg = ' '.join((str(m) for m in msg))
        print 'UAV: %s' % msg


    def error(self, *msg):
        msg = ' '.join((str(m) for m in msg))
        print 'UAV error: %s' % msg




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
            return None
        return self.uav


    def get_states(self):
        return _vehicle2states(self.uav)


    def get_position(self):
        states = self.get_states()
        if states:
            return Position(
                lat = states.location_global_frame_lat,
                lon = states.location_global_frame_lon,
                alt = states.location_global_frame_alt)
        return None




    @state(TARGETSET)
    def set_target(self, pos):

        self.target = pos

        self.uav.commands.clear()

        cur = self.get_position()
        cmd = dronekit.Command(0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 1, 0, 0, 0, 0, cur.lat, cur.lon, cur.alt)
        self.uav.commands.add(cmd)

        cmd = dronekit.Command(0, 0, 0,
           mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
           0, 0, 0, 0, 0, 0,
           pos.lat, pos.lon, pos.alt)
        self.uav.commands.add(cmd)

        self.uav.commands.upload()
        return True


    @state(PREARMED)
    def prearm(self):
        """UAV prearm check.
        """
        self.log('Prearm')

        self.uav.mode = dronekit.VehicleMode('GUIDED')

        #cnt = 0
        #while not self.uav.is_armable:
        #    self.log('Waiting until UAV is armable...')
        #    cnt += 1
        #    if cnt > UAV_TIMEOUT_PREARM:
        #        self.log('timeout armable')
        #        return False
        #    time.sleep(1)
        #self.log('is armable')

        cnt = 0
        while self.uav.gps_0.fix_type < 2:
            self.log('Waiting for UAV GPS...')
            cnt += 1
            #if cnt > TIMEOUT_GPS:
            #    self.log('UAV timeout gps'
            #    return False

        self.log('GPS fix:', self.uav.gps_0.fix_type)
        return True


    @state(ARMED)
    def arm(self):
        self.log('Arm')

        self.uav.mode = dronekit.VehicleMode('GUIDED') # ?

        self.uav.armed = True
        cnt = 0
        while not self.uav.armed:
            self.log('Waiting until UAV is armed...')
            if cnt > UAV_TIMEOUT_ARM:
                self.log('UAV timeout arm')
                return False
            time.sleep(1)

        self.log('UAV is armed')
        return True


    @state(LAUNCHED)
    def launch(self):
        self.log('Launch')

        self.uav.simple_takeoff(self.target.alt)
        return True


    @state(LANDED)
    def land(self):
        self.log('Land')

        self.uav.mode = dronekit.VehicleMode('LAND')
        self.uav.flush()
        return True


    @state(DISARMED)
    def disarm(self):
        self.log('Disarm')

        self.uav.armed = False
        self.uav.flush()
        return True


uav = UAV()


def _vehicle2states(vehicle):
    if not vehicle:
        return None

    states = {
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

    class DictToObject:
        def __init__(self, d):
            self.__dict__ = d
    return DictToObject(states)

