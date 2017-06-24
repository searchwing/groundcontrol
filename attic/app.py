# -*- coding: utf-8 -*-
"""Start server with
'uwsgi --http :8000 --wsgi-file server.py'
"""
from cgi import route       # To route http requests py path
from cgi import background  # To move execution into a background threat
from cgi import application # Required import: The uwsgi application object

from cgi import render_to_response # Convenience function to render html and start 200 response.



@route('/')
def index(start_response):
    return render_to_response(
            start_response, {'message': 'hello world',}, 'index.html')

