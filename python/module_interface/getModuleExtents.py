"""
Tool for walking a user through getting all of the module extents
for a chain of modules.  It also helps by writing the "module_params"
section of a result file if you want to specify one.

Run with --help for more details.

NOTE: You probably need someone to help you with this.  One
      person to hold the modules at +/-90 deg and another to
      hit <Enter> to capture.  Get a friend!

NOTE 2: This was written straight up stream of consciousness.  Beware.
"""
from ModuleIface import ModuleIface
from argparse import ArgumentParser
from sys import exit
from os.path import isfile
from time import sleep
import json, shutil
from pprint import pprint

ap = ArgumentParser(description="Tool for manually finding the extents of a number of modules (whose brain board ids you must specify)")
ap.add_argument('dev',
                type=str,
                help="The serial device (probably a file in /dev) we should use to talk to modules.")
ap.add_argument('--bbids',
                type=int,
                nargs='+',
                required=True,
                help="Specify which brain board ids we should get the extents of")
ap.add_argument('--file','-f',
                type=str,
                help="A result json file to load (or create) and add 'module_params' entry in top level 'results' dict so subsequent analyzeResults.py calls will have module_params.")
ap.add_argument('--debug','-d',
                action="store_true",
                default=False,
                help="Turn on debugging output.")

def usage(msg):
    """
    Print argparse help message with a helpful message describing what
    we think is the problem.
    """
    ap.print_usage()
    print "-"*40
    print msg
    exit(1)

def killCAN(mIface, bbid):
   """
   Try to force the module with brain board ID = bbid to stop
   passing CAN messges through.
   """
   for i in xrange(100):
       mIface.can_pass(bbid, ModuleIface.CAN_NONE)

def affirmative(msg):
    """
    Print msg and then see if the response is 'y' or 'Y'.  Return
    True if so, False otherwise.
    """
    ans = raw_input(msg + " (enter 'y' or 'Y' for yes): ")
    if ans in ('y', 'Y'):
        return True
    else:
        return False

def getPos(mIface, bbid):
    """
    Get the current position of the module with brain board id bbid.
    """
    mIface.flush()
    mIface.can_pass(bbid, ModuleIface.CAN_ALL)
    print "Before: ", len(mIface.feedback)
    sleep(0.2)
    killCAN(mIface, bbid)
    mIface.update()
    print "After: ", len(mIface.feedback)

    posSum = reduce(lambda x,y: x+y, [fb.pos_raw for fb in mIface.feedback])
    pos = int(posSum / len(mIface.feedback))
    return pos

def getCalibInteractive(mIface, bbid):
    """
    Interactively get the min, max, and middle
    Q15 readings from a module with the brain board
    id bbid using mIface to talk to modules.

    RETURNS:
        [min_tics, max_tics, mid_tics]
    """
    while True:
        calib = list()
        print "Ready to get the calibration (extents) for module with Brain Board ID=%d" % bbid
        print "Make sure to hold the module still each time you need to hit <Enter>!"
        one = raw_input("  Move the module to its MINIMUM angle, then hit <Enter>.")
        calib.append(getPos(mIface, bbid))
        two = raw_input("  Move the module to its MAXIMUM angle, then hit <Enter>.")
        calib.append(getPos(mIface, bbid))
        three = raw_input("  Move the module to some position between MIN and MAX, then hit <Enter>.")
        calib.append(getPos(mIface, bbid))
        if affirmative("We got [min, max, mid] of [%d,%d,%d] for bbid=%d, is this alright?" % (calib[0], calib[1], calib[2], bbid)):
            return calib
        else:
            print "Arrrg!  Alright, trying again for bbid=%d" % bbid

args = ap.parse_args()
args.bbids = set(args.bbids)

if args.file and isfile(args.file):
    print "The file (%s) already exists.  Loading its json..." % args.file
    with open(args.file) as f:
        json_out = json.load(f)
else:
    json_out = json.loads('{}')
    
if json_out.has_key('results'):
    if json_out['results'].has_key('module_params'):
        inp = raw_input("The json file (%s) already has a ['results']['module_params'] section in it!\n\
                         If you want to overwrite it, hit <Enter>.  Otherwise, hit ctrl-d to quit." % args.file)
else:
    json_out['results'] = {}
    json_out['results']['module_params'] = {}

mIface = ModuleIface(args.dev, debug=args.debug)

for bbid in args.bbids:
    killCAN(mIface, bbid)

knownBbids = mIface.scan(args.bbids)
if not args.bbids.issubset(knownBbids):
    print "Cannot see some modules you asked for."
    print "      I can see: ", knownBbids
    print "  You asked for: ", args.bbids

    missing = args.bbids.difference(args.bbids.intersection(knownBbids))
    print "        Missing: ", missing 
    inp = raw_input("Hit enter to continue and only work with those module's we've seen, or hit <ctrl-d> to quit")

# The old motor controllers won't start sending feedbacks until
# it gets at least one command.  There's no way to tell the brainboard
# to send a pwm(0) command to the motor controller other than starting
# the trajectory...
# So print crazy warnings, and tell them all to run their trajectories for 1 second.
print "-"*40
print "WARNING WARNING WARNING WARNING WARNING....."
print "  We need to tell all of the modules to run a 1 second trajectory"
print "  so that the motor controllers will start updating their feedback."
print "  "
print "  If you have loaded trajectories, they will be run!  Power cycle"
print "  the modules to initalize all of their trajectories to 0s."
moveOnAndDie = raw_input("Hit <Enter> to send the 1 second trajectory, or ctrl-d to quit.")
mIface.start_traj(1)

# Get the calibrations
finBbids = args.bbids.intersection(knownBbids)
for bbid in finBbids:
    json_out['results']['module_params'][str(bbid)] = getCalibInteractive(mIface, bbid)

print "The calibration we found is: "
print "  ['results']['module_params'] = "
pprint(json_out['results']['module_params'])

if args.file and affirmative("Do you want me to write this back to file (%s)?" % args.file):
    if isfile(args.file):
        backFile = args.file+".backup"
        print "Creating backup of original file: %s" % backFile
        shutil.copy(args.file, backFile)
    with open(args.file,'w') as f:
        f.write(json.dumps(json_out))
