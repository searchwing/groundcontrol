# -*- coding: utf-8 -*-
"""Async work on serial devices.
"""
import serial, time, threading, traceback


class SerialThread(threading.Thread):
    """Abstract superclass for threads working on serial devices.
    """

    def __init__(self, name, port, baud, timeout = 10):
        super(SerialThread, self).__init__()
        self.name = name
        self.port, self.baud, self.timeout = port, baud, timeout

        self.cond = threading.Condition()
        self.daemon = True
        self.ser = None


    def log(self, *msg):
        msg = ' '.join((str(m) for m in msg))
        print '%s: %s' % (self.name, msg,)
        

    def wait(self, timeout = None):
        self.cond.acquire()
        self.cond.wait(timeout = timeout)
        self.cond.release()


    def notify(self):
        self.cond.acquire()
        self.cond.notifyAll()
        self.cond.release()


    def run(self):
        while 1:
            try:
                if not self.ser:
                    self.log('Open', self.port, self.baud)
                    self.ser = serial.Serial()
                    self.ser.port     = self.port
                    self.ser.baudrate = self.baud
                    self.ser.timeout  = self.timeout
                    self.ser.open()
                    self.log('Is open')

                self.work()

            except serial.SerialException, e:
                self.log('Serial error', e)
            except Exception, e:
                self.log('Unkown error', e)
                traceback.print_exc()
            finally:
                if self.ser:
                    self.log('Close')
                    try:
                        self.ser.close()
                    except:
                        pass
                    finally:
                        self.ser = None
                    self.log('Is closed')
                time.sleep(2)


    def work(self):
        raise Exception('Not implemented')

