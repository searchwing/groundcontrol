# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
"""Global condition to sync everything.
"""
import threading


# global confition
_condition = threading.Condition()


def wait(timeout = None):
    """Wait until passed timeout or someone calls notify.
    """
    global _condition

    _condition.acquire()
    _condition.wait(timeout = timeout)
    _condition.release()


def notify():
    """Notify all threats waiting in wait.
    """
    global _condition

    _condition.acquire()
    _condition.notifyAll()
    _condition.release()
