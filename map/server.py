#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read from serial gps, open an http server, provide a map.
"""


VENV_DIR = '/Users/sascha/venv'


if __name__ == '__main__':

    if VENV_DIR:
        activate_this = '%s/bin/activate_this.py' % VENV_DIR
        execfile(activate_this, dict(__file__ = activate_this))

    import os, sys
    import cherrypy

    PATH = os.path.dirname(os.path.realpath(__file__))

    sys.path.append(os.path.join(PATH, '..'))
    from swgc import gps


#_template = 'map_ol3.html'
_template = 'map_mapbox.html'


_html = open(_template).read()

_gps = None


class Server(object):

    @cherrypy.expose
    def index(self):
        pos = _gps.get_position()
        if pos:
            lat, lon = pos.lat, pos.lon
        else:
            lat, lon = 0, 0
        _html = open(_template).read()
        return _html #% (lon, lat)

    
    @cherrypy.expose
    @cherrypy.tools.json_out()
    def pos(self):
        pos = _gps.get_position()
        if pos:
            lat, lon = pos.lat, pos.lon
        else:
            lat, lon = 0, 0
        return {'geometry': {'type': 'Point', 'coordinates': [lon, lat,]}, 'type': 'Feature',}# 'properties': {},}


def main():
    global _gps

    _gps = gps.GPS(name = 'gps', port = sys.argv[1], baud = 4800)
    _gps.start()

    config = {
        'global' : {
            'server.socket_host' : '127.0.0.1',
            'server.socket_port' : 8080,
            'server.thread_pool' : 8,
            'tools.staticdir.content_types' : {
                'css'  : 'text/css',
                'js'   : 'application/javascript',
                'json' : 'application/json',
                'pbf'  : 'application/binary',
             },
        },
        '/static' : {
            'tools.staticdir.on'  : True,
            'tools.staticdir.dir' : os.path.join(PATH, 'static'),
            'tools.gzip.on'      : True,
            'tools.expires.on'   : True,
            'tools.expires.secs' : 86400,
        },
        '/tiles' : {
            'tools.staticdir.on'  : True,
            'tools.staticdir.dir' : os.path.join(PATH, 'static', 'tiles'),
            'tools.gzip.on'      : True,
            'tools.expires.on'   : True,
            'tools.expires.secs' : 86400,
        }
    }
    cherrypy.quickstart(Server(), config = config)


if __name__ == '__main__':
    main()
