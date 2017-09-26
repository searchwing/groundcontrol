#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @author: sascha@searchwing.org, August 2017
"""Main.
See notes for (how to install) requirements.
See swgc.settings for settings.
"""

if __name__ == '__main__':

    # Activate python virtualenv in current user homedir.
    import os
    activate_this = '%s/venv/bin/activate_this.py' % os.path.expanduser('~')
    execfile(activate_this, dict(__file__ = activate_this))

    # Run app
    from swgc import app
    try:
        app.run()
    except KeyboardInterrupt:
        # Intentional killed
        print 'Ctrl-C'
    
