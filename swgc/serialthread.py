# -*- coding: utf-8 -*-
"""Async work on serial devices.
"""
import serial, time, threading, traceback

from . import sync




class SerialThread(threading.Thread):
    """Abstract superclass for threads working on serial devices.
    Notifies global app lock on successful connection.
    Subclasses have to implement work(), it will be called after
    successfull connecting the serial device, when work() returns
    the serial device will be closed and reopened again.
    """

    def __init__(self, name, port, baud, timeout = 10):
        """Init.
        """
        super(SerialThread, self).__init__()
        self.name = name
        self.port, self.baud, self.timeout = port, baud, timeout

        self.daemon = True
        self.ser = None


    def log(self, *msg):
        """Some logging.
        """
        msg = ' '.join((str(m) for m in msg))
        print '%s: %s' % (self.name, msg,)


    def run(self):
        """Internal. Don't call.
        """
        while 1:
            connection_failed = True # Expecting the worst
            try:
                # Connect
                self.log('Open', self.port, self.baud)
                self.ser = serial.Serial(
                        port     = self.port,
                        baudrate = self.baud,
                        timeout  = self.timeout)

                # Open
                if self.ser.isOpen(): # Can it be open already? Lets test, better is better
                    self.log('Already open')
                else:
                    self.ser.open()
                    self.log('Is open')

                connection_failed = False # No exception, we have a connection

                sync.notify() # Global notify

                self.work() # Expected to be blocking, afer this the port will be closed again

            except serial.SerialException, e:
                self.log('Serial error', e)
            except Exception, e:
                self.log('Unkown error', e)
                traceback.print_exc()

            if self.ser:
                self.log('Close')
                try:
                    self.ser.close()
                except:
                    pass

                self.ser = None
                self.log('Is closed')

            if connection_failed:
                # Don't freak out, sleep a second
                time.sleep(1)


    def work(self):
        """To be implemented by inheriting classes.
        After return the serial device will be closed again.
        Dont call, it will be called by this thread.
        """
        raise Exception('Not implemented')
