#!/usr/bin/env python
import time, threading
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, WaypointPush


NAME      = 'pilot'
LOG_NAME  = '%s_log'  % NAME
NODE_NAME = '%s_node' % NAME

TOPIC_NAME_STATE         = '/mavros/state'
TOPIC_NAME_GLOBALPOSITON = '/mavros/global_position/raw/fix'

# timeout for rospy.wait_for_service, must be None or > 0
WAIT_FOR_SERVICE_TIMEOUT = 1

MODE_MANUAL     = 'MANUAL'
MODE_GUIDED     = 'GUIDED'
MODE_STABILIZED = 'STABILIZED'
MODE_LOITER     = 'AUTO.LOITER'
MODE_MISSION    = 'AUTO.MISSION'


#logpublisher = rospy.Publisher(LOG_NAME, String, queue_size = 10)

positionPublisher = rospy.Publisher(
        '/mavros_plane/setpoint_raw/global',
        GlobalPositionTarget, queue_size = 10)

setmodeService = rospy.ServiceProxy('/mavros/set_mode',    SetMode)
armingService  = rospy.ServiceProxy('/mavros/cmd/arming',  CommandBool)
takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
landService    = rospy.ServiceProxy('/mavros/cmd/land',    CommandTOL)
wpPushService  = rospy.ServiceProxy('mavros/mission/push', WaypointPush)


# For asynchronus notifications
_condition = threading.Condition()


_running = True
_fix     = None
_state   = None




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




def notify():
    """Notify all threats waiting for local condition.
    """
    _condition.acquire()
    _condition.notifyAll()
    _condition.release()




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




def waitForMode(mode, timeout = None):
    """Wait for passed mode.
    """
    Info("Wait for mode '%s', timeout %s", mode, timeout)

    def testfunc():
        if not _state:
            return False
        return _state.mode == mode
    ret = waitFor(testfunc, timeout) 

    Info("Wait for mode '%s' timeout %s returns %s", mode, timeout, ret)
    return ret




def waitForArmed(armed, timeout = None):
    """Wait for passed armed/disarmed.
    """
    Info('Wait for armed %s, timeout %s' , armed, timeout)

    def testfunc():
        if not _state:
            return False
        return _state.armed == armed
    ret = waitFor(testfunc, timeout) 

    Info("Wait for armed %s timeout %s returns %s", armed, timeout, ret)
    return ret




def stateCallback(state):
    """Callback for state changes.
    Notifies waiting threats.
    """
    global _state

    changed = not _state                                or not \
            _state.system_status == state.system_status or not \
            _state.mode          == state.mode          or not \
            _state.connected     == state.connected     or not \
            _state.armed         == state.armed         or not \
            _state.guided        == state.guided

    if changed:
        Info('State changed\n  from %s\n  to   %s',
            stateToSring(_state), stateToSring(state))

    _state = state
    if changed:
        notify()




def globalPositionCallback(fix):
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




def shutdownHook():
    """Shut down callback.
    Notifies waiting threats.
    """
    global _running
    Info('Shutdown %s', NAME)

    _running = False
    notify()




def setMode(mode):
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
        rospy.wait_for_service('/mavros/set_mode',
                timeout = WAIT_FOR_SERVICE_TIMEOUT)
        ret = setmodeService(custom_mode = mode)

    except rospy.ROSException, err:
        Info("Set mode to '%s' call failed with error '%s'", mode, err)
        return None

    ret = ret.mode_sent
    Info("Set mode to '%s' returns %s", mode, ret)
    return ret




def arm(arm):
    """Arm/Disarm. Pass True for arm, False for disarm.
    Return True/False for Success, Failure.
    Return None for rospy.ROSException
    """
    try:
        rospy.wait_for_service('/mavros/cmd/arming',
                timeout = WAIT_FOR_SERVICE_TIMEOUT)
        ret = armingService(arm)

    except rospy.ROSException, err:
        Info("%s call failed with error '%s'",
                ('Arm' if arm else 'Disarm'), err)
        return None

    ret = ret.success
    Info('%s returns %s', ('Arm' if arm else 'Disarm'), ret)
    return ret




def takeoff(altitude):
    """Takeoff.
    Return True/False for Success, Failure.
    Return None for rospy.ROSException
    """
    try:
        rospy.wait_for_service('/mavros/cmd/takeoff',
                timeout = WAIT_FOR_SERVICE_TIMEOUT)
        ret = takeoffService(
                latitude = _fix.latitude, longitude = _fix.longitude,
                altitude = altitude,
                min_pitch = 0, yaw = 0)

    except rospy.ROSException, err:
        Info("Takeoff call failed with error '%s'", err)
        return None

    ret = ret.success
    Info('Takeoff returns %s', ret)
    return ret




def land():
    """Land.
    Return True/False for Success, Failure.
    Return None for rospy.ROSException
    """
    try:
        rospy.wait_for_service('/mavros/cmd/land',
                timeout = WAIT_FOR_SERVICE_TIMEOUT)
        ret = landService(
            altitude = 0, latitude = 0, longitude = 0,
            min_pitch = 0, yaw = 0)

    except rospy.ROSException, err:
        Info("Land call failed with error '%s'", err)
        return None

    ret = ret.success
    Info('Land returns %s', ret)
    return ret




def setPosition(fix):
    """Set position.
    """
    # http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-global-int
    target = GlobalPositionTarget(
            type_mask        = 0b0000111111111000,
            coordinate_frame = 6,
            latitude         = fix.latitude,
            longitude        = fix.longitude,
            altitude         = fix.altitude)
    return target




def menu():
    """Commandline text menu.
    """
    print
    print 'Press'
    print '1: to set mode to GUIDED'
    print '2: to set mode to STABILIZED'
    print '3: to set mode to ARM'
    print '4: to set mode to DISARM'
    print '5: to set mode to TAKEOFF'
    print '6: to set mode to LAND'
    print '7: to print position'
    print '8: to print state'
    print '9: to test'
    print 'q: to exit'
    print 'or enter a string for a mode to set'
    print
    choice = raw_input('Enter your input: ');
    print
    return choice


def main():
    """Main loop.
    """
    # Needed for logpublisher, for logging we must be a node
    print('Init node %s' % NODE_NAME) # Regular loggin g not yet available
    # No timeout for this?
    rospy.init_node(NODE_NAME, anonymous = False, log_level = rospy.DEBUG)
    Info('Node %s initialized', NODE_NAME)

    # Register shutdown hook
    # In this case we don't really have a use for this hook
    rospy.on_shutdown(shutdownHook)

    # Register state callback
    rospy.Subscriber(TOPIC_NAME_STATE, State,
            callback = stateCallback)

    # Register position callback
    rospy.Subscriber(TOPIC_NAME_GLOBALPOSITON, NavSatFix,
            callback = globalPositionCallback)

    Info('Running %s, going into main loop', NAME)
    # A custom main loop, alternatively to rospy.spin()
    while not rospy.is_shutdown():
        choice = menu()

        if choice == 'q':
            print 'Bye!'
            break

        if   choice == '1':
            if setMode(MODE_GUIDED):
                waitForMode(MODE_GUIDED, 5)

        elif choice == '2':
            if setMode(MODE_STABILIZED):
                waitForMode(MODE_STABILIZED, 5)

        elif choice == '3':
            if arm(True):
                waitForArmed(True, 5)

        elif choice == '4':
            if arm(False):
                waitForArmed(False, 5)

        elif choice == '5':
            takeoff(5)

        elif choice == '6':
            land()

        elif choice == '7':
            if _fix:
                print 'longitude: %f' % _fix.longitude, \
                      'latitude:  %f' % _fix.latitude, \
                      'altitude:  %d' % _fix.altitude
            else:
                print 'No position available'

        elif choice == '8':
            Info(stateToSring(_state))

        elif choice == '9':
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                pos = setPosition(_fix)
                positionPublisher.publish(pos)
                rate.sleep()

        elif choice:
            if setMode(choice):
                waitForMode(choice)#, 5)




if __name__ == '__main__':
    main() # looping
