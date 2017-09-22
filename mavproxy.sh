#!/usr/bin/env bash
DEV=/dev/serial0
OUT="udp:192.168.5.181:14550"
#OUT="udpbcast:192.168.5.255:14550"
. /opt/venv/bin/activate && mavproxy.py $DEV --out=$OUT --daemon
