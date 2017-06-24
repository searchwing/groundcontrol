# -*- coding: utf-8 -*-
"""Provide DB functions.
"""
import sqlite3
from settings import *


_db = None


def init():
    """Internal.
    """
    global db
    print 'Connect DB'

    _db = sqlite3.connect(DATABASE)
    cur = _db.cursor()

    cur.execute('SELECT SQLITE_VERSION()')
    print 'Connected DB sqlite', cur.fetchone()[0]

    print 'Create tables'
    try:
        Mission.create_table()
        Waypoint.create_table()
    except sqlite3.OperationalError, e:
        print 'Seems tables are already created ("...%s...")' % e




def db():
    return _db




def cursor():
    return _db.cursor()




class Mission:

    def __init__(self, name, id = None, ts = None, waypoints = []):
        self.id        = id
        self.name      = name
        self.ts        = ts if not ts == None else datetime.utcnow()
        self.waypoints = waypoints if not waypoints == None else []


    def save(self):
        cur = cursor()


    @staticmethod
    def create_table():
        cur = cursor()
        cur.execute(
            'create table missions(id int primary key, name text not null unique, ts unsigned big int not null)')


    @staticmethod
    def for_name(name):
        cur = cursor()
        cur.execute("select * from missions where name='%s'" % name)




class Waypoint:

    def __init__(self, mission, lat, lon, height, id = None):
        self.id      = id
        self.mission = mission
        self.lat     = lat
        self.lon     = lon
        self.height  = height


    def save(self):
        cur = cursor()


    @staticmethod
    def create_table():
        cur = cursor()
        cur.execute(
            'create table waypoints(id int primary key, lat real not null, lon real not null, mission int not null, foreign key(mission) references missions(id))')

