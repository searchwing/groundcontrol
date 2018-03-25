#!/usr/bin/env python3
# Joystick to gimbal: the joystick
import sys, time
import serial
from math import sqrt
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import commands as cm


DEBUG = False

ADDRESS, PORT = '0.0.0.0', 8000

httpd, gimbal = None, None




class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        ret = 200 if handle_cmd(self.path) else 400
        self.send_response(ret)




def handle(msg):
    msg = msg.split('/')
    if not msg:
        return False
    msg = msg[1:]
    if not msg:
        return False
    for m in msg:
        m = m.split('_')
        if not m:
            return False
        cmd, args = m[0], m[1:]
        if not args:
            return False
        if not move_cmd(cmd, args):
            return False
    return True


def cmd_gimbal(cmd, args):
    if cmd == 'dir':
        args = [int(arg, 16) for arg in args]
        YSH, YSL, PSH, PSL = args
        gimbal.command(gimbal.make_cmd(
            pitchmode=cm.mode_speed,
            yawmode=cm.mode_speed,
            PSH=PSH, PSL=PSL,
            YSH=YSH, YSL=YSL))
        return True
    return False




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




def main():
    global gimbal, httpd
    print('Starting gimbal')

#    gimbal = Gimbal()
#    gimbal.connect()
#    gimbal.command(cm.home)

    print('Starting httpd')
    httpd = HTTPServer((ADDRESS, PORT), Handler)
    httpd.serve_forever()




if __name__ == '__main__':
    while 1:
        try:
            main()
        except KeyboardInterrupt, err:
            print 'Interrupt, Exit 0'
            break
        except Exception, err:
            print err
