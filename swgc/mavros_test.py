#!/usr/bin/env python
import time, threading
from geo import Position

import mavros, rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget, Waypoint, WaypointList
from mavros_msgs.srv import SetMode, CommandHome, CommandBool, CommandTOL, WaypointPush, WaypointClear


NAME      = 'pilot'
LOG_NAME  = '%s_log'  % NAME
NODE_NAME = '%s_node' % NAME

TOPIC_STATE             = '/mavros/state'
TOPIC_POSITON_FIX       = '/mavros/global_position/raw/fix'
TOPIC_MISSION_WAYPOINTS = '/mavros/mission/waypoints'

# default timeout for rospy.wait_for_service, wait for ever with None or 0
WAIT_FOR_MAVLINK_TIMEOUT = 10 

MODE_MANUAL     = 'MANUAL'
MODE_GUIDED     = 'GUIDED'
MODE_STABILIZED = 'STABILIZED'
MODE_LOITER     = 'AUTO.LOITER'
MODE_MISSION    = 'AUTO.MISSION'

# Tbd: find this constants in rospy
MAV_CMD_NAV_WAYPOINT          = 16
MAV_CMD_NAV_TAKEOFF           = 22
MAV_CMD_NAV_LAND              = 21
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3


#logpublisher = rospy.Publisher(LOG_NAME, String, queue_size = 10)


setmodeService      = rospy.ServiceProxy('/mavros/set_mode',     SetMode)
sethomeService      = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
armingService       = rospy.ServiceProxy('/mavros/cmd/arming',   CommandBool)
takeoffService      = rospy.ServiceProxy('/mavros/cmd/takeoff',  CommandTOL)
landService         = rospy.ServiceProxy('/mavros/cmd/land',     CommandTOL)
missionClearService = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
wpPushService       = rospy.ServiceProxy('mavros/mission/push',  WaypointPush)


# For asynchronus notifications
_condition = threading.Condition()


_running   = True
_fix       = None
_state     = None
_waypoints = None



# Logging -->

def Info(*args):
    """Info logging.
    Internal wrapper for rospy.loginfo.
    """
    rospy.loginfo(*args)
    #message = args[0] % args[1:] if len(args) > 1 else args[0]
    #logpublisher.publish(message)


def stateToSring(state):
    """Return a string for passed mavros state.
    """
    if state:
        return "system_status:%s connected:%s armed:%s guided:%s mode:'%s'" % (
            state.system_status, state.connected,
            state.armed, state.guided, state.mode)
    return None

# <--



# Wait, Notify -->

def waitFor(testfunc, timeout = None):
    """Wait for passed testfunc evaluating to True.
    """
    _condition.acquire()

    if timeout:
        ts = time.time()
    while _running:

        ok = testfunc()
        if ok:
            return ok

        if timeout:
            elapsed = time.time() - ts
            if elapsed >= timeout:
                return ok
            _condition.wait(timeout - elapsed)
        else:
            _condition.wait()

    _condition.release()


def notify():
    """Notify all threats waiting for local condition.
    """
    _condition.acquire()
    _condition.notifyAll()
    _condition.release()

# <--




# Init -->

def init():
    """Init ROS, register subscribers, publishers, ...
    """
    print('Init node %s' % NODE_NAME) # Regular loggin g not yet available

    mavros.set_namespace()
    rospy.init_node(NODE_NAME, anonymous = False, log_level = rospy.DEBUG)
    Info('Node %s initialized', NODE_NAME)

    # Register shutdown hook
    rospy.on_shutdown(shutdownHook)

    # Register state callback
    rospy.Subscriber(TOPIC_STATE,
        State, callback = stateCallback)

    # Register position callback
    rospy.Subscriber(TOPIC_POSITON_FIX,
        NavSatFix, callback = positionCallback)

    # Register waypoint callback
    rospy.Subscriber(TOPIC_MISSION_WAYPOINTS,
        WaypointList, callback = missionCallback)

# <--




# Callbacks -->

def shutdownHook():
    """Shut down callback.
    Notifies waiting threats.
    """
    global _running
    Info('Shutdown %s', NAME)

    _running = False
    notify()


def stateCallback(state):
    """Callback for state changes.
    Notifies waiting threats.
    """
    global _state, _fix

    changed = not _state                                or not \
            _state.system_status == state.system_status or not \
            _state.mode          == state.mode          or not \
            _state.connected     == state.connected     or not \
            _state.armed         == state.armed         or not \
            _state.guided        == state.guided

    if not state or not state.connected:
        _fix = None

    if changed:
        Info('State changed\n  from %s\n  to   %s',
            stateToSring(_state), stateToSring(state))

    _state = state
    if changed:
        notify()


def positionCallback(fix):
    """Callback for global position.
    Notifies waiting threats.
    """
    global _fix

    if not _fix:
        changed = True
    else:
        lat1, lon1 = _fix.latitude, _fix.longitude
        lat2, lon2 =  fix.latitude,  fix.longitude
        lat1, lon1 = int(lat1 * 1000), int(lon1 * 1000)
        lat2, lon2 = int(lat2 * 1000), int(lon2 * 1000)
        alt1, alt2 = _fix.altitude, fix.altitude
        alt1, alt2 = int(alt1 * 10), int(alt2 * 10)
        changed = not lat1 == lat2 or \
                  not lon1 == lon2 #or \
                  #not alt1 == alt2
    _fix = fix

    if changed:
        Info('Position changed\n  to  latitude:%s longitude:%s altitude:%s',
                _fix.latitude, _fix.longitude, _fix.altitude)
        notify()


def missionCallback(waypoints):
    """Callback for set waypoints.
    Notifies waiting threats.
    """
    global _waypoints
    _waypoints = waypoints

    Info('Waypoints now: waypoints:%s, current:%s',
            len(waypoints.waypoints) if _waypoints else None,
            waypoints.current_seq    if _waypoints else None)
    notify()

# <--




# wait for connected, position -->

def waitForConnected(timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Wait for connected mavlink device.
    """
    Info('Wait for connection timeout %s' , timeout)

    def testfunc():
        if not _state:
            return False
        return _state.connected
    ret = waitFor(testfunc, timeout) 

    Info("Wait for connection timeout %s returned %s", timeout, ret)
    return ret


def waitForPosition(timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Wait for connected mavlink device.
    """
    Info('Wait for position timeout %s' , timeout)

    def testfunc():
        if not _fix:
            return False
        return True
    ret = waitFor(testfunc, timeout) 

    Info("Wait for position timeout %s returned %s", timeout, ret)
    return ret



# Arm, Disarm, Wait for armed/disarmed -->

def arm(arm, timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Arm/Disarm. Pass True for arm, False for disarm.
    Return True/False for Success, Failure.
    Return None for rospy.ROSException
    """
    try:
        armingService.wait_for_service(timeout)
        ret = armingService(arm)

    except rospy.ROSException, err:
        Info("%s call failed with error '%s'",
                ('Arm' if arm else 'Disarm'), err)
        return None

    ret = ret.success
    Info('%s call returned %s', ('Arm' if arm else 'Disarm'), ret)
    return ret


def waitForArmed(armed, timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Wait for passed armed/disarmed.
    """
    Info('Wait for armed %s timeout %s' , armed, timeout)

    def testfunc():
        if not _state:
            return False
        return _state.armed == armed
    ret = waitFor(testfunc, timeout) 

    Info("Wait for armed %s timeout %s returned %s", armed, timeout, ret)
    return ret

# <--




# Set mode, Wait for mode -->

def setMode(mode, timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Set custom mode.
    See http://wiki.ros.org/mavros/CustomModes for custom modes
    mavros log output gives us the available modes
        AUTO.FOLLOW_TARGET
        AUTO.RTGS
        AUTO.LAND
        AUTO.RTL
        AUTO.MISSION
        RATTITUDE
        AUTO.LOITER
        STABILIZED
        AUTO.TAKEOFF
        OFFBOARD
        POSCTL
        ALTCTL
        AUTO.READY
        ACRO
        MANUAL
    Return True/False for Success, Failure.
    Return None for rospy.ROSException
    """
    try:
        setmodeService.wait_for_service(timeout = timeout)
        ret = setmodeService(custom_mode = mode)

    except rospy.ROSException, err:
        Info("Set mode to '%s' call failed with error '%s'", mode, err)
        return None

    ret = ret.mode_sent
    Info("Set mode to '%s' call returned %s", mode, ret)
    return ret


def waitForMode(mode, timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Wait for passed mode.
    """
    Info("Wait for mode '%s' timeout %s", mode, timeout)

    def testfunc():
        if not _state:
            return False
        return _state.mode == mode
    ret = waitFor(testfunc, timeout) 

    Info("Wait for mode '%s' timeout %s returned %s", mode, timeout, ret)
    return ret

# <--




# Set, clear, wait for waypoints -->

def setMission(positions, timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Upload mission of passed positions.
    """
    wl = WaypointList()

    pos = positions[0]

    wp = Waypoint()
    wp.command      = MAV_CMD_NAV_TAKEOFF
    wp.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
    wp.is_current   = True
    wp.autocontinue = True
    wp.x_lat        = pos.lat
    wp.y_long       = pos.lon
    wp.z_alt        = pos.alt
    wl.waypoints.append(wp)

    for pos in positions[1:]:

        wp = Waypoint()
        wp.command      = MAV_CMD_NAV_WAYPOINT
        wp.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp.is_current   = False
        wp.autocontinue = True
        wp.x_lat        = pos.lat
        wp.y_long       = pos.lon
        wp.z_alt        = pos.alt
        wl.waypoints.append(wp)

    wp = Waypoint()
    wp.command      = MAV_CMD_NAV_LAND
    wp.frame        = MAV_FRAME_GLOBAL_RELATIVE_ALT
    wp.is_current   = False
    wp.autocontinue = True
    wp.x_lat        = pos.lat
    wp.y_long       = pos.lon
    wp.z_alt        = 0
    wl.waypoints.append(wp)

    try:
        wpPushService.wait_for_service(timeout = timeout)
        ret = wpPushService(0, wl.waypoints)

    except rospy.ROSException, err:
        Info("Waypoint push call failed with error '%s'", err)
        return None

    ret = ret.success
    Info('Waypoint push call returned %s', ret)
    return ret


def clearMission():
    """Clear current mission.
    """
    global waypoints

    _waypoints = None

    try:
        ret = missionClearService.call()

    except rospy.RosException, err:
        Info("Clear mission call failed with error '%s'", err)
        return None

    ret = ret.success
    Info('Clear mission call returned %s', ret)
    return ret


def waitForMisson(beSet, timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Wait for passed set/cleared misson.
    """
    Info('Wait for mission timeout %s', timeout)

    if beSet:
        def testfunc():
            return True  if _waypoints and _waypoints.waypoints else False
    else:
        def testfunc():
            return False if _waypoints and _waypoints.waypoints else True

    ret = waitFor(testfunc, timeout) 

    Info("Wait for mission %s timeout %s returned %s", beSet, timeout, ret)
    return ret

# <---




# commands -->

def sethome(timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Set current position as home position.
    """
    try:
        ret = sethomeService(current_gps = True,
            latitude  = _fix.latitude,
            longitude = _fix.longitude,
            altitude  = _fix.altitude)

    except rospy.ROSException, err:
        Info("set home call failed with error '%s'", err)
        return None

    ret = ret.success
    Info('set home call returned %s', ret)
    return ret


def takeoff(altitude, timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Takeoff.
    Return True/False for Success, Failure.
    Return None for rospy.ROSException
    """
    try:
        takeoffService.wait_for_service(timeout = timeout)
        ret = takeoffService(
                latitude = _fix.latitude, longitude = _fix.longitude,
                altitude = altitude,
                min_pitch = 0, yaw = 0)

    except rospy.ROSException, err:
        Info("Takeoff call failed with error '%s'", err)
        return None

    ret = ret.success
    Info('Takeoff call returned %s', ret)
    return ret


def land(timeout = WAIT_FOR_MAVLINK_TIMEOUT):
    """Land.
    Return True/False for Success, Failure.
    Return None for rospy.ROSException
    """
    try:
        landService.wait_for_service(timeout = timeout)
        ret = landService(
            altitude = 0, latitude = 0, longitude = 0,
            min_pitch = 0, yaw = 0)

    except rospy.ROSException, err:
        Info("Land call failed with error '%s'", err)
        return None

    ret = ret.success
    Info('Land call returned %s', ret)
    return ret




def testMission():
    """Set a test mission.
    """
    global _waypoints

    while not waitForConnected():
        pass

    while not waitForPosition():
        pass

    pos0 = Position(_fix.latitude, _fix.longitude, _fix.altitude + 10)
    pos1 = pos0.get_location_by_offset_meters_and_heading(10, 45) # 10m in 45degree
    mission = [pos0, pos1, pos0,]

    if setMission(mission):
        waitForMisson(True)
    return

    if \
    \
    arm(False)                   and \
    waitForArmed(False)          and \
    \
    setMode(MODE_STABILIZED)     and \
    waitForMode(MODE_STABILIZED) and \
    \
    sethome()                    and \
    \
    clearMission()               and \
    waitForMisson(False)         and \
    \
    setMission(mission)          and \
    waitForMisson(True)          and \
    \
    setMode(MODE_MISSION)        and \
    waitForMode(MODE_MISSION)    and \
    \
    arm(True)                    and \
    waitForArmed(True):
        takeoff(10)


def main():
    """Main loop.
    """
    def menu():
        """Commandline text menu.
        """
        print
        print 'Press'
        print 's: to set mode to STABILIZED'
        print 'h: to set home'
        print 'a: to arm'
        print 'd: to disarm'
        print 't: to takeoff'
        print 'l: to land'
        print 'c: to clear mission'
        print 'm: test mission'
        print 'p: to print state and position'
        print 'q: to exit'
        print 'or enter a string for a mode to set'
        print
        choice = raw_input('Enter your input: ');
        print
        return choice

    init()

    Info('Running %s, going into main loop', NAME)
    # A custom main loop, alternatively to rospy.spin()
    while not rospy.is_shutdown():
        choice = menu()

        if choice == 'q':
            print 'Bye!'
            break

        elif choice == 's':
            if setMode(MODE_STABILIZED):
                waitForMode(MODE_STABILIZED, 5)

        elif choice == 'h':
            sethome()

        elif choice == 'a':
            if arm(True):
                waitForArmed(True, 5)

        elif choice == 'd':
            if arm(False):
                waitForArmed(False, 5)

        elif choice == 't':
            takeoff(10)

        elif choice == 'l':
            land()

        elif choice == 'c':
            if clearMission():
                waitForMisson(False, 5)

        elif choice == 'm':
            testMission()

        elif choice == 'p':
            print stateToSring(_state)
            if _fix:
                print 'latitude: %f'  % _fix.latitude,  \
                      'longitude: %f' % _fix.longitude, \
                      'altitude: %d'  % _fix.altitude
            else:
                print 'No position'

        elif choice:
            if setMode(choice):
                waitForMode(choice)#, 5)




if __name__ == '__main__':
    main() # looping
