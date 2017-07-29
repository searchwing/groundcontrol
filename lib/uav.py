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
    """

    def __init__(self, *args, **kwargs):
        super(self.__class__, self).__init__(*args, **kwargs)
        self.daemon = True

        self.msg = None

        self.last_heartbeat = 0
        self.uav            = None
        self.states         = {}
        self.target         = None




    def log(self, *msg):
        self.msg = ' '.join((str(m) for m in msg))
        print 'UAV: %s' % self.msg


    def err(self, *msg):
        self.msg = ' '.join((str(m) for m in msg))
        print 'UAV error: %s' % self.msg


    def get_message(self):
        return self.msg or 'Stand by...'




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
                    #self.uav.add_attribute_listener(
                    #        '*', self.on_any)

            if self.uav and not UAV_TIMEOUT_CONNECTION:
                break

            time.sleep(UAV_TIMEOUT_CONNECTION_TEST)


    def on_heartbeat(self, *args, **kwargs):
        self.last_heartbeat = time.time()


    def on_any(self, uav, name, value):
        self.states[name] = value




    def get_uav(self):
        return self.uav


    def set(self, key, val):
        self.log('Set %s: %s' % (key, val))

        if not self.uav:
            self.err('No UAV connected. Cant set')
            return False

        self.uav.parameters[key] = val
        return True


    def get_settings(self):
        if not self.uav:
            #self.err('No UAV connected. Cant get settings')
            return None

        return self.uav.parameters


    def get_states(self):
        if not self.uav:
            self.err('No UAV connected. Cant get settings')
            return None

        return _vehicle2states(self.uav)


    def get_position(self):
        if not self.uav:
            #self.err('No UAV connected. Cant get position')
            return None

        lat = self.uav.location.global_frame.lat
        lon = self.uav.location.global_frame.lon
        alt = self.uav.location.global_frame.alt

        if lat == None or lon == None or alt == None:
            return None
        return Position( lat = lat, lon = lon, alt = alt)




    def set_target(self, pos):
        self.log('Set target', pos)

        if not self.uav:
            self.err('No UAV connected. Cant set target')
            return False

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

        self.target = pos
        return True


    def get_target(self):
        return self.target




    def _prearm(self):
        """UAV prearm check.
        """
        self.log('Prearm')

        self.uav.mode = dronekit.VehicleMode('GUIDED')

        cnt = 0
        while not self.uav.is_armable:
            self.log('Waiting until UAV is armable...')
            cnt += 1
            if cnt > UAV_TIMEOUT_PREARM:
                self.log('timeout armable')
                return False
            time.sleep(1)
        self.log('Is armable')

        cnt = 0
        while self.uav.gps_0.fix_type < 2:
            self.log('Waiting for UAV GPS...')
            cnt += 1
            if cnt > TIMEOUT_GPS:
                self.log('UAV timeout gps')
                return False

        self.log('GPS fix:', self.uav.gps_0.fix_type)
        return True


    def arm(self):
        self.log('Arm')

        if not self.uav:
            self.err('No UAV connected. Cant arm')
            return False

        if not self._prearm():
            pass
            #return False

        self.uav.mode = dronekit.VehicleMode('GUIDED') # ?

        self.uav.armed = True
        cnt = 0
        while not self.uav.armed:
            self.log('Waiting until UAV is armed...')
            cnt += 1
            if cnt > UAV_TIMEOUT_ARM:
                self.log('UAV timeout arm')
                return False
            time.sleep(1)

        self.log('Armed')
        return True


    def disarm(self):
        self.log('Disarm')

        if not self.uav:
            self.err('No UAV connected. Cant disarm')
            return False

        self.uav.armed = False
        self.uav.flush()
        return True


    def is_armed(self):
        if not self.uav:
            self.err('No UAV connected. Dont know if armed.')
            return False

        return self.uav.armed


    def launch(self):
        self.log('Launch', self.target.alt)

        if not self.uav:
            self.err('No UAV connected. Cant launch')
            return False

#        if not self.uav.armed:
#            self.err('Not armed. Cant launch')
#            self.arm()

        self.uav.simple_takeoff(self.target.alt)
        return True


    def land(self):
        self.log('Land')

        if not self.uav:
            self.err('No UAV connected. Cant land')
            return False

        self.uav.mode = dronekit.VehicleMode('LAND')
        self.uav.flush()
        return True


    def return_to_launch(self):
        self.log('Return to launch')

        if not self.uav:
            self.err('No UAV connected. Cant rtl')
            return False

        self.uav.mode = dronekit.VehicleMode("RTL")
        self.uav.flush()
        return True


uav = UAV()


def _vehicle2states(vehicle):
    class vstates:
        def __init__(self, vehicle):
            self.version_major             = vehicle.version.major             if vehicle else None
            self.version_patch             = vehicle.version.patch             if vehicle else None
            self.version_release           = vehicle.version.release           if vehicle else None
            self.system_status             = vehicle.system_status.state       if vehicle else None
            self.is_armable                = vehicle.is_armable                if vehicle else None
            self.armed                     = vehicle.armed                     if vehicle else None
            self.gps_fix                   = vehicle.gps_0.fix_type            if vehicle else None
            self.gps_sats                  = vehicle.gps_0.satellites_visible  if vehicle else None
            self.location_global_frame_alt = vehicle.location.global_frame.alt if vehicle else None
            self.location_global_frame_lon = vehicle.location.global_frame.lon if vehicle else None
            self.location_global_frame_lat = vehicle.location.global_frame.lat if vehicle else None
            self.rangefinder_distance      = vehicle.rangefinder.distance      if vehicle else None
            self.rangefinder_voltage       = vehicle.rangefinder.voltage       if vehicle else None
            self.attitude_pitch            = vehicle.attitude.pitch            if vehicle else None
            self.attitude_roll             = vehicle.attitude.roll             if vehicle else None
            self.attitude_yaw              = vehicle.attitude.yaw              if vehicle else None
            self.velocity                  = vehicle.velocity                  if vehicle else None
            self.airspeed                  = vehicle.airspeed                  if vehicle else None
            self.groundspeed               = vehicle.groundspeed               if vehicle else None
            self.heading                   = vehicle.heading                   if vehicle else None
            self.battery_voltage           = vehicle.battery.voltage           if vehicle else None
            self.battery_current           = vehicle.battery.current           if vehicle else None
            self.battery_level             = vehicle.battery.current           if vehicle else None
            self.ast_heartbeat             = vehicle.last_heartbeat            if vehicle else None
    return vstates(vehicle)

