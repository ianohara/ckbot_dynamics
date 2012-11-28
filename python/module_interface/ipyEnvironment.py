"""
This script is meant to be run from within ipython in order
to setup all of the components of our system for interactive
use.

USAGE:
  Run Ipython then at Ipython prompt:
    In [1]: run ipyEnvironment.py

  Follow instructions from there...
"""
print "Importing TrajLoader, PositionLogger, and ModuleIface Classes."
from trajLoader import TrajLoader, exceedRC710TorqueFunc
from posTest import PositionLogger
from moduleIface import ModuleIface
import json

dev = raw_input("What serial device should be used [/dev/ttyUSB0]? ")
if not dev:
    dev = '/dev/ttyUSB0'

jsonDat = json.loads('{}')
test_time = 0.1
debug=True

print "Creating ModuleIface instance 'mi' using device=%s" % dev
mi = ModuleIface(dev)

print "Creating PositionLogger instance 'pl' using bullshit values."
pl = PositionLogger(modIface=mi, test_time=test_time, jsonout=jsonDat,
debug=debug)

