#!/usr/bin/env python3

import serial
import time
import sys

import pygame

from termcolor import colored

import commands as cm

from math import sqrt


class Gimbal():
    def __init__(self, tty='/dev/ttyUSB0', sleep=0.012):
        self.ser = None
        self.tty = tty
        self.baud_rate = 115200
        self.sleep = sleep

    def connect(self):
        self.ser = serial.Serial(self.tty,
                                 self.baud_rate,
                                 serial.EIGHTBITS,
                                 serial.PARITY_NONE,
                                 serial.STOPBITS_ONE,
                                 rtscts=False,
                                 timeout=3,
                                 xonxoff=False,
                                 dsrdtr=False,
                                 interCharTimeout=None)
        if not self.ser.isOpen():
                print("could not open serial device")
                sys.exit(0)
        else:
            self.ser.flush()
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.sendBreak(duration=0.25)
        #self.command(cm.home)
        time.sleep(self.sleep)

    def close(self):
        if self.ser.isOpen():
            self.ser.close()
        else:
            print('device was not open')

    def __stripheader(self, header, seq):
        if not header or not seq:
            return seq
        if header[0] == seq[0]:
            return self.__stripheader(header[1:], seq[1:])
        else:
            return self.__stripheader(header, seq[1:])

    def get_position(self):
        result = [int(x) for x in self.__stripheader([0x3e, 0x3d, 0x36, 0x73], self.command(cm.status, 59))]
        result = result[4:]
        for i in range(0, len(result)):
            print("%3d" % i, end=" ")
        print("")
        for i in range(0, len(result)):
            print("%3d" % result[i], end=" ")
        print("")


    def command(self, cmd, num=0):
            if not self.ser:  # and not self.ser.isOpen:
                self.connect()
            self.ser.flush()
            self.ser.write(cmd)
            self.ser.flushOutput()
            if num:
                return self.ser.read(num)
            else:
                return self.ser.read_all()


    def make_cmd(self,
                 rollmode=cm.mode_no_control,
                 pitchmode=cm.mode_no_control, #cm.mode_angle_rel_frame,
                 yawmode=cm.mode_no_control, #cm.mode_angle_rel_frame,
                 RSL=0x00,
                 RSH=0x00,
                 RAL=0x00,
                 RAH=0x00,
                 PSL=0x00,
                 PSH=0x00,
                 PAL=0x00,
                 PAH=0x00,
                 YSL=0x00,
                 YSH=0x00,
                 YAL=0x00,
                 YAH=0x00):
        result = [rollmode, pitchmode, yawmode, RSL, RSH, RAL, RAH, PSL, PSH, PAL, PAH, YSL, YSH, YAL, YAH]
        cmd = cm.pl_header + result + [sum(result) % 256]
        #print(cmd)
        cmd_c = colored(' '.join([hex(cmd[0]), hex(cmd[1]), hex(cmd[2]), hex(cmd[3])]), 'blue') + \
                ' ' + colored(' '.join([hex(cmd[4]), hex(cmd[5]), hex(cmd[6])]), 'red') + \
                ' ' + colored(' '.join([hex(cmd[7]), hex(cmd[8])]), 'yellow') + \
                ' ' + colored(' '.join([hex(cmd[9]), hex(cmd[10])]), 'green') + \
                ' ' + colored(' '.join([hex(cmd[11]), hex(cmd[12])]), 'cyan') + \
                ' ' + colored(' '.join([hex(cmd[13]), hex(cmd[14])]), 'magenta') + \
                ' ' + colored(' '.join([hex(cmd[15]), hex(cmd[16])]), 'blue', 'on_white') + \
                ' ' + colored(' '.join([hex(cmd[17]), hex(cmd[18])]), 'red', 'on_white') + \
                ' ' + colored(hex(cmd[19]), 'yellow', 'on_white')
        #print(cmd_c)
        return cmd


if __name__ == '__main__':
    pygame.init()
    gimbal = Gimbal(tty="/dev/ttyUSB0")
    print("instantiated") 
    #print(gimbal.get_postion())
    gimbal.connect()
    print("connected")
    gimbal.command(cm.home)

    done = False

    mode = cm.day_mode

    if not pygame.joystick.get_count():
        print("no joystick found")
        sys.exit()



    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    YAW_AXIS=0
    PITCH_AXIS=1
    RATE_AXIS=2
    THRESHOLD=0.1
    SLEEP=0.01
    
    while not done:
        time.sleep(SLEEP)
        bts = [hex(x) for x in gimbal.ser.read_all()]
        if bts:
            print(" ".join(bts))
            pass
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
	            done=True
            if event.type == pygame.JOYHATMOTION:
                direction = event.value[1]
                if direction == 1:
                    # Zoom in
                    print("zoom in")
                    gimbal.command(cm.zoom_in)
                elif direction == -1:
                    print("zoom out")
                    # Zoom out
                    gimbal.command(cm.zoom_out)
                else:
                    gimbal.command(cm.zoom_stop)
                    #time.sleep(0.1)
                    #print([hex(x) for x in gimbal.ser.read(100)])
                    time.sleep(SLEEP)
                    direction = event.value[0]
                    if direction == 1:
                        print("focus in")
                        gimbal.command(cm.focus_in)
                    elif direction == -1:
                        print("focus out")
                        gimbal.command(cm.focus_out)
                    else:
                        gimbal.command(cm.focus_stop)

            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    print("click")
                    gimbal.command(mode)
                    if mode == cm.day_mode:
                        mode = cm.night_mode
                    else:
                        mode = cm.day_mode
                elif event.button == 1:
                    print("CMD STATUS")
                    #gimbal.command([0x3e, 0x19, 0x32, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00, 00])
                    gimbal.command([0x3e, 0x19, 0x01, 0x1a, 0xff, 0xff])
                    #print(gimbal.command(cm.status))
                elif event.button == 2:
                    print("Yangda documentation, get zoom position")
                    print(gimbal.command(cm.get_zoom_pos))
                elif event.button == 3:
                    print("query position")
                    gimbal.get_position()
                else:
                    print("PELCO D zoom query (not working)")
                    print(gimbal.command(cm.zoom_query))

                #time.sleep(1)
                #gimbal.get_position()
            elif True: #event.type == pygame.JOYAXISMOTION:
                yaw_value = joystick.get_axis(YAW_AXIS)
                pitch_value = joystick.get_axis(PITCH_AXIS)
                # use throttle to control sensitivity
                rate_value = ((joystick.get_axis(RATE_AXIS)+1.1)/2.1) ** 2
                # override with less extreme settings for now
                rate_value = ((joystick.get_axis(RATE_AXIS)+1.1)/2.1) ** 1
                #print(yaw_value)
                if yaw_value > THRESHOLD:
                    YSH = 0x00
                    YSL = int(255 * abs(yaw_value) * rate_value)
                elif -yaw_value > THRESHOLD:
                    YSH = 0xff
                    # signed little endian my ass
                    # FIXME, FUCKING UGLY
                    YSL = 0xff & (255 - int(255 * abs(yaw_value) * rate_value))
                else:
                    YSL = 0
                    YSH = 0
                if -pitch_value > THRESHOLD:
                    PSH = 0x00
                    PSL = int(255 * abs(pitch_value) * rate_value)
                elif pitch_value > THRESHOLD:
                    PSH = 0xff
                    PSL = 0xff & (255 - int(255 * abs(pitch_value) * rate_value))
                else:
                    PSL = 0
                    PSH = 0 
                gimbal.command(gimbal.make_cmd(#rollmode=cm.mode_speed,
					    pitchmode=cm.mode_speed,
					    yawmode=cm.mode_speed,
					    PSH=PSH, PSL=PSL,
                                            YSH=YSH, YSL=YSL))
                                            #YAH=0x00, YAL=0x00))
    gimbal.command(cm.stop)



