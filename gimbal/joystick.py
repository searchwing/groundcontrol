#!/usr/bin/env python
# Joystick to gimbal: the joystick
import time
import pygame, requests


DEBUG = True

ADDRESS, PORT = '0.0.0.0', 8000
GIMBAL_URL = 'http://%s:%s' % (ADDRESS, PORT)

YAW_AXIS   = 0
PITCH_AXIS = 1
RATE_AXIS  = 2
THRESHOLD  = 0.1
SLEEP      = 0.01


def call_gimbal(msg):
    if DEBUG:
        print msg
    requests.get('%s%s' % (GIMBAL_URL, msg))


def main():
    print 'Starting'
    print

    pygame.init()
     
    if not pygame.joystick.get_count():
        print 'No joystick found'
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print 'Found a joystick (Using the first connected)'
    print

    mode = 'mode_day'
    
    while 1:
        time.sleep(SLEEP)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
	        break

            msg = ''

            if event.type == pygame.JOYHATMOTION:
                direction = event.value[1]
                if direction == 1:
                    msg = '%s/cmd_zoomin'   % msg

                elif direction == -1:
                    msg = '%s/cmd_zoomout'  % msg

                else:
                    msg = '%s/cmd_zoomstop' % msg
                    time.sleep(SLEEP)

                    direction = event.value[0]
                    if direction == 1:
                        msg = '%s/cmd_focusin'   % msg

                    elif direction == -1:
                        msg = '%s/cmd_focusout'  % msg

                    else:
                        msg = '%s/cmd_focusstop' % msg

            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    msg = '%s/%s' % (msg, mode)
                    if mode == 'mode_day':
                        mode = 'mode_night'
                    else:
                        mode = 'mode_day'

                elif event.button == 1:
                    msg = '%s/%s' % (msg,
                        ','.join(str(v) for v in [0x3e, 0x19, 0x01, 0x1a, 0xff, 0xff]))

            else:
                yaw_value   = joystick.get_axis(YAW_AXIS)
                pitch_value = joystick.get_axis(PITCH_AXIS)
                rate_value  = ((joystick.get_axis(RATE_AXIS)+1.1)/2.1) ** 2
                rate_value  = ((joystick.get_axis(RATE_AXIS)+1.1)/2.1) ** 1

                if yaw_value     > THRESHOLD:
                    YSH = '0x00'
                    YSL = str(int(255 * abs(yaw_value) * rate_value))

                elif -yaw_value  > THRESHOLD:
                    YSH = '0xff'
                    YSL = str(0xff & (255 - int(255 * abs(yaw_value) * rate_value)))

                else:
                    YSH = '0x00'
                    YSL = '0x00'

                if -pitch_value  > THRESHOLD:
                    PSH = '0x00'
                    PSL = str(int(255 * abs(pitch_value) * rate_value))

                elif pitch_value > THRESHOLD:
                    PSH = '0xff'
                    PSL = str(0xff & (255 - int(255 * abs(pitch_value) * rate_value)))

                else:
                    PSH = '0x00'
                    PSL = '0x00'

                msg = '%s/%s' % (msg, 'dir')

                msg = '%s_%s' % (msg, YSH)
                msg = '%s_%s' % (msg, YSL)
                msg = '%s_%s' % (msg, PSH)
                msg = '%s_%s' % (msg, PSL)

            call_gimbal(msg)

    msg = 'cmd_stop'
    call_gimbal(msg)


if __name__ == '__main__':
    while 1:
        try:
            main()
        except KeyboardInterrupt, err:
            print 'Interrupt, Exit 0'
            break
        except Exception, err:
            print err
            print 'Napping for 2 seconds'
            time.sleep(2)
