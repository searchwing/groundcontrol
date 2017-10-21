# -*- coding: utf-8 -*-
"""Async work on serial devices.
"""
import time, threading, traceback, serial




class SerialThread(threading.Thread):
    """Abstract superclass for threads working on serial devices.
    Subclasses have to implement work()
    """

    def __init__(self, name, port, baud, timeout = 10):
        """Init.
        """
        super(SerialThread, self).__init__()
        self.daemon = True

        self.name = name
        self.port, self.baud, self.timeout = port, baud, timeout
        self.ser, self.running = None, False


    def start(self):
        """Start.
        """
        self.log('Start')
        self.running = True

        super(SerialThread, self).start()


    def close(self):
        """Close serial connection if any.
        """
        self.log('Close')
        self.running = False

        ser, self.ser = self.ser, None
        if not ser is None:
            try:
                ser.close()
            except BaseException, e:
                pass # We can't do anything here


    def run(self):
        """Internal. Don't call.
        """
        while self.running:
            try:
                # Connect
                self.log('Open', self.port, self.baud)
                ser = serial.Serial(
                    port     = self.port,
                    baudrate = self.baud,
                    timeout  = self.timeout)

                # Open
                if ser.isOpen(): # Can it be open already? Lets test, better is better
                    self.log('Already open')
                else:
                    ser.open()
                    self.log('Is open')

                self.ser = ser

                # work() is supposed to be blocking until its finished
                # If it returns False the port will be closed and the serial thread finished.
                # Else the port will be reopened and the loop continues.
                if not self.work():
                    self.stop()

            except serial.SerialException, e:
                self.log('Serial error', e)

            except BaseException, e:
                self.log('Unkown error', e)
                traceback.print_exc()

            if self.running:
                # Don't freak out, sleep a second
                time.sleep(1)


    def work(self):
        """To be implemented by inheriting classes.
        Dont call, it will be called by this thread.
        Is is supposed to be blocking until its finished.
        If it returns False the port will be closed and the serial thread finishes,
        Else the port will be reopened and the loop continues.
        """
        raise Exception('Not implemented')


    def log(self, *msg):
        """Some logging.
        """
        msg = ' '.join((str(m) for m in msg))
        print '%s: %s' % (self.name, msg,)


    def __str__(self):
        """To string.
        """
        return self.name
