#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL


LOG_NAME  = 'groundcontrol_log'
NODE_NAME = 'groundcontrol_node'

TOPIC_NAME_STATE         = '/mavros/state'
TOPIC_NAME_GLOBALPOSITON = '/mavros/global_position/raw/fix'

MODE_GUIDED     = 'GUIDED'
MODE_STABILIZED = 'STABILIZED'


#logpublisher = rospy.Publisher(LOG_NAME, String, queue_size = 10)


positionPublisher  = rospy.Publisher(
        '/mavros_plane/setpoint_raw/global',
        GlobalPositionTarget, queue_size = 10)

setmodeService = rospy.ServiceProxy('/mavros/set_mode',    SetMode)
armingService  = rospy.ServiceProxy('/mavros/cmd/arming',  CommandBool)
takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
landService    = rospy.ServiceProxy('/mavros/cmd/land',    CommandTOL)


fix = None


def Info(*args):
    """Info logging.
    Internal wrapper for rospy.loginfo.
    """
    rospy.loginfo(*args)
    #message = args[0] % args[1:] if len(args) > 1 else args[0]
    #logpublisher.publish(message)


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
    Return None for rospy.ServiceException
    """
    rospy.wait_for_service('/mavros/set_mode')
    try:
        ret = setmodeService(custom_mode = mode)
    except rospy.ServiceException, err:
        Info("Set mode to '%s' call failed with error %s", mode, err)
        return None
    ret = ret.mode_sent
    Info("Set mode to '%s' call returns %s", mode, ret)
    return ret


def arm(arm):
    """Arm/Disarm. Pass True for arm, False for disarm.
    Return True/False for Success, Failure.
    Return None for rospy.ServiceException
    """
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        ret = armingService(arm)
    except rospy.ServiceException, err:
        Info('%s call failed with error: %s' ('Arm' if arm else 'Disarm'), err)
        return None
    ret = ret.success
    Info('%s call returns %s', ('Arm' if arm else 'Disarm'), ret)
    return ret


def takeoff():
    """Takeoff.
    Return True/False for Success, Failure.
    Return None for rospy.ServiceException
    """
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        ret = takeoffService(
                latitude = 0, longitude = 0, altitude = 2,
                min_pitch = 0, yaw = 0)
    except rospy.ServiceException, err:
        Info('Takeoff call failed with error %s', err)
        return None
    ret = ret.success
    Info('Takeoff call returns %s', ret)
    return ret


def land():
    """Land.
    Return True/False for Success, Failure.
    Return None for rospy.ServiceException
    """
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        ret = landService(
            altitude = 0, latitude = 0, longitude = 0,
            min_pitch = 0, yaw = 0)
    except rospy.ServiceException, err:
        Info('Land call failed with error %s', err)
        return None
    ret = ret.success
    Info('Land call returns %s', ret)
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


def globalPositionCallback(globalPosition):
    """Callback for global position.
    Register with
    rospy.Subscriber(TOPIC_NAME_GLOBALPOSITON,
            NavSatFix, globalPositionCallback)
    """
    global fix
    fix = globalPosition




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
    print '7: print GPS coordinates'
    print '8: testing'
    print 'Or enter a string for a mode to set,'
    print 'Or just hit ENTER to quit.'
    print
    choice = raw_input('Enter your input: ');
    print
    return choice


def main():
    """Main loop.
    """
    # Needed for logpublisher, for logging we must be a node
    #rospy.init_node(NODE_NAME, anonymous = True)

    rospy.Subscriber(TOPIC_NAME_GLOBALPOSITON,
            NavSatFix, globalPositionCallback)

    while not rospy.is_shutdown():
        x = menu()

        if not x:
            print 'Bye'
            break

        if   x == '1':
            setMode(MODE_GUIDED)
        elif x == '2':
            setMode(MODE_STABILIZED)

        elif x == '3':
            arm(True)
        elif x == '4':
            arm(False)

        elif x == '5':
            takeoff()
        elif x == '6':
            land()

        elif x == '7':
            if fix:
                print 'longitude: %.7f' % fix.longitude, \
                      'latitude:  %.7f' % fix.latitude
            else:
                print 'No position available'

        elif x == '8':
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                pos = setPosition(fix)
                positionPublisher.publish(pos)
                rate.sleep()

        else:
            setMode(x)




if __name__ == '__main__':
    main() # looping
