#!/usr/bin/python
from trajLoader import TrajLoader, exceedRC710TorqueFunc
from posTest import PositionLogger
from ModuleIface import ModuleIface
from os.path import isfile

import json, time, sys

def usage(msg):
    print "Usage: %s <dev> <trajectory file> [<module list>]" % sys.argv[0]
    print "  Where <trajectory file> contains a 'control' json  dictionary entry containing a"
    print "  dictionary specifying the trajectory. If [<module list>] is not specified"
    print "  the json file should also contain a 'modules' key which has a list value of "
    print "  module ids in order from the chain base to chain tip."
    print "-"*80
    print msg
    sys.exit(1)

if len(sys.argv) < 3:
    usage()

dev = sys.argv[1]
if not isfile(dev):
   usage("Device file does not exist! (%s)" % dev)
trajFile = sys.argv[2]

with open(trajFile, 'r') as jsfh:
    jsonDat = json.load(jsfh)

modlist = None
if len(sys.argv) == 4:
    modlist = json.load(sys.argv[3])
    if not isinstance(modlist, list):
        usage("The modlist should be a python/json list of module ids.")

name = "%s_%s" % (trajFile, time.strftime("result_%a-%b-%d-%Y_%H-%M-%S"))
print "Using result file name of: ", name

test_time = max([step["end_time"] for step in jsonDat["controls"]])
test_time = test_time + 2 # Capture for 2 seconds after end of trajectory
print "Using test time of: ", test_time

debug=True

mIface = ModuleIface(dev, debug=debug)
print "WARN: Using Torque->PWM Conversion function fo ExceedRC710KV Motor ONLY!"
trajL = TrajLoader(module_iface=mIface,
                   control_json=jsonDat,
                   module_map=modlist,
                   torque_func=exceedRC710TorqueFunc,
                   debug=debug)

pTest = PositionLogger(modIface=mIface,
                       test_time=test_time,
                       jsonout=jsonDat,
                       test_name=name,
                       debug=debug)

print "Writing trajectory to the modules..."
if not trajL.write_trajectory():
    print "Could not load the trajectory!"
    sys.exit(1)

raw_input("When ready to run the test, hit enter...")
# This runs the test and then writes it to the file specified by
# the test_name argument to the constructor...in this case
# our "name" variable.
pTest.run_test()
