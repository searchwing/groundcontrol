#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Main.
See notes for (how to install) requirements.
See swgc.settings for settings.
"""

VENV_DIR = '/home/pi/venv/'


if __name__ == '__main__':

    # Activate python virtualenv if any
    if VENV_DIR:
        activate_this = '%s/bin/activate_this.py' % VENV_DIR
        execfile(activate_this, dict(__file__ = activate_this))

    # Run the app
    from swgc import app
    app.run()
