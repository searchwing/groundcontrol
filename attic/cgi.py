# -*- coding: utf-8 -*-
"""Http server functions.
"""
import os, traceback, threading
from jinja2 import Template


TEMPLATEPATH = 'templates' # Must be relative to this file


_path2function = {}
_templatepath  = os.path.abspath(
        os.path.join(os.path.dirname(__file__), TEMPLATEPATH))




def application(env, start_response):
    """The wsgi app.
    """
    path = env['PATH_INFO']
    routed = _path2function.get(path)
    if not routed:
        start_response('400', [('Content-Type', 'text/plain'),])
        return ('404 Not Found',)
    return routed(env, start_response)




def route(path):
    """Decorator to route http requests and pass query parameters.
    """
    def decorator(to_decorate):
        def decorated(env, start_response):
            args = env['QUERY_STRING'].split('&')
            args = [arg.split('=') for arg in args if arg]
            args = {arg[0]:arg[1] for arg in args}
            try:
                return to_decorate(start_response, **args)
            except Exception, e:
                print e
                traceback.print_exc()
                start_response(
                    '500 Server Error', [('Content-Type', 'text/plain'),])
                return (str(e),)
        _path2function[path] = decorated
    return decorator




def render_to_response(start_response, context, template):
    """Convenience function to render html and start 200 response.
    """
    template = get_template(template)
    ret = template.render(context)
    ret = str(ret)
    start_response('200 OK', [('Content-Type', 'text/html'),])
    return (ret,)




def get_template(template):
    """Get template for name.
    Todo: Cache templates.
    """
    path = os.path.join(_templatepath, template)
    template = open(path).read()
    return Template(template)



def background(run):
    """Run passed function in a thread.
    """
    thread = threading.Thread(target = run)
    thread.daemon = True
    thread.start()

