#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Test.
Test the swgc.uav module.
"""


VENV_DIR = '/Users/sascha/venv/'


if __name__ == '__main__':

    # Activate python virtualenv if any
    if VENV_DIR:
        activate_this = '%s/bin/activate_this.py' % VENV_DIR
        execfile(activate_this, dict(__file__ = activate_this))

    import sys
    connection_string = sys.argv[1] if len(sys.argv) > 1 else None

    # Run the uav test.
    from swgc import uav_test
    uav_test.run(connection_string = connection_string)
