#!/usr/bin/python
from math import pi
import numpy, sys, time as Time, json

import matplotlib.pyplot as plt

from Energy import ETree

module_traj = {}

module_param = {3: [18880, 2350, 27216],
                4: [24868, 8530, 250],
                5: [23152, 6614, 31600]}

module_order = {4: 0, 5: 1, 3: 2} # xbee id -> base to tip order number

def usage():
    print sys.argv[0], " <test data file> <cpp sim results file> [time offset]"

if (len(sys.argv) < 3):
    usage()
    exit(1)

test_name = sys.argv[1]#'tests/swing_test_magic'
sim_result_file = sys.argv[2]

time_shift=0.0
if (len(sys.argv) == 4):
    time_shift = float(sys.argv[3])

e = ETree(file=sim_result_file)
damping = e._ETree__results["chain"][0]["damping"]

test_dat = json.load(open( test_name, 'r' ))
dat = test_dat['data']
test_time = test_dat['time']
test_name = test_dat['name']
modules = test_dat['modules']

for k,v in module_param.items():
    offs = ((2**15/4-v[1])+(2**15*3/4-v[0]))/2
    module_param[k].append(offs)

for dat_stub in dat:
    module_id = dat_stub[0]
    pos_raw = dat_stub[1]
    if pos_raw > 2**14:
        pos_raw = pos_raw - 2**15

    max_tics = module_param[module_id][1]
    min_tics = module_param[module_id][0] - 2**15
    pos = -pi/2 + (pi/2--pi/2)/(max_tics-min_tics)*(pos_raw-min_tics)

    time = dat_stub[2]
    if time < 0:
        continue
    #if time > test_time*1000:
    #    break

    voltage = dat_stub[3]
    if not module_traj.has_key(module_id):
        module_traj[module_id] = []
    module_traj[module_id].append([time, pos])

plt.hold(True)
plt.grid(True)
plt.axis([min(e.time), max(e.time), -1.57, 1.57])
plt.xlabel("Time [s]")
plt.ylabel("Radians [rad/s]")
legStrs = list()
for module in modules.values():
    t_p = numpy.array(module_traj[module])
    plt.plot((t_p[...,0]/1000.0 - time_shift),
             t_p[...,1], 'o', markersize=2.0)
    print module, " is mod number."
    legStrs.append("Phys Mod %d" % module)


for index in xrange(e.module_count):
    plt.plot([t for t in e.time],
             [s[index] for s in e.states],
             "-", linewidth=1.0)
    legStrs.append("Sim Mod %d" % index)

plt.legend(legStrs)
save_file = "images/physical_verification/%s_damping%f_shift%f_epochtime%d.png" % (
             sim_result_file.replace("..", "").replace("/","-"),
             damping, time_shift, int(Time.time())
             )

print "Saving this plot to ", save_file

plt.savefig(save_file, dpi=480)
plt.show()
