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
import json

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

print "Trying to turn on heartbeat pass for brain boards: %s" % args.bbids
for bbid in args.bbids:
    mIface.can_pass(bbid, ModuleIface.CAN_HB)
print "Now waiting for 5 seconds to make sure we can see all of the modules..."
knownBbids = mIface.discover(timeout=5.0)

if not args.bbids.issubset(knownBbids):
    print "Cannot see some modules you asked for."
    print "      I can see: ", knownBbids
    print "  You asked for: ", args.bbids

    missing = args.bbids.difference(args.bbids.intersection(knownBbids))
    print "        Missing: ", missing 
    inp = raw_input("Hit enter to continue and only work with those module's we've seen, or hit <ctrl-d> to quit")

finBbids = args.bbids.intersection(knownBbids)

def getCalibInteractive(mIface, bbid):
    """
    Interactively get the min, max, and middle
    Q15 readings from a module with the brain board
    id bbid using mIface to talk to modules.
    """

for bbid in finBbids:
    json_out['results']['module_params'][bbid] = getCalibInteractive(mIface, bbid)
