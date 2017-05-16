# -*- coding: utf-8 -*-
"""Provide DB functions.
"""
import sqlite3

from settings import *



_db = None

def init(): 
    global _db
    """
    Open SQL DB.
    Call on app initialization, and only once.
    """
    print 'Connect DB'
    _db = sqlite3.connect(DATABASE)

