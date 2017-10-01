#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read from serial gps, open an http server, provide a map.
"""

VENV_DIR = '/Users/sascha/venv'


if __name__ == '__main__':

    if VENV_DIR:
        activate_this = '%s/bin/activate_this.py' % VENV_DIR
        execfile(activate_this, dict(__file__ = activate_this))

    import sys, time
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
    import SocketServer
    import serial
    from swgc import gps


html = """<!DOCTYPE html>
<html>
<script src="https://openlayers.org/en/v4.3.4/build/ol.js"></script>
<div id="map" style="width: 600px, height: 400px"></div>
<script>
  var map = new ol.Map({
    layers: [
      new ol.layer.Tile({source: new ol.source.OSM()})
    ],
    view: new ol.View({
      center: ol.proj.transform([%s, %s], 'EPSG:4326','EPSG:3857'),
      zoom: 5,
    }),
    target: 'map'
  });
  var update = function(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/pos', true);
    xhr.responseType = 'json';
    xhr.onload = function(){
      var status = xhr.status;
      if (status === 200){
        var res = xhr.response;
        var lat = res.lat;
        var lon = res.lon;
        map.getView().setCenter(ol.proj.transform([lon, lat], 'EPSG:4326','EPSG:3857'));
      }
    };
    xhr.send();
  };
  setInterval(update, 2000);
</script>
</html>
"""


_gps = None


class HttpdHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        pos = _gps.get_position()
        if pos:
            lat, lon = pos.lat, pos.lon
        else:
            lat, lon = 0, 0

        path = self.path[1:]

        if not path:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            content = html % (lon, lat)
            self.wfile.write(content)

        elif path == 'pos':
            self.send_response(200)
            self.send_header('Content-type', 'text/json')
            self.end_headers()
            content = '{"lat":%s, "lon":%s}' % (lat, lon)
            self.wfile.write(content)


def main():
    global _gps
    _gps = gps.GPS(name = 'gsp', port = sys.argv[1], baud = 4800)
    _gps.start()

    httpd = HTTPServer(('', 8000), HttpdHandler)
    httpd.serve_forever()


if __name__ == '__main__':
    main()
