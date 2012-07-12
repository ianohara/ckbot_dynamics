#!/usr/env/bin python

import moduleIface as mI
from time import time as now, sleep

feedback = []
iface = mI.moduleIface('/dev/ttyUSB0')

modules = [38]

for m in modules:
    print 'Setting param'
    iface.set_param_sync( m, 'fdbkRate', 100 )

# ['dt', 'voltage']
cmd = 80
traj = [[0   ,  0],
	[2.0 ,  cmd],
	[4.0 , -cmd]]

sleep( 0.5 )

# Execute trajectory
iface.flush()
ind = 0
t0 = now()
while True:
    iface.update()
    curtime = now()
    if curtime - t0 >  traj[ind][0]:
	print "time: %f, cmd: %s" % ( curtime-t0, repr(traj[ind][1]))
	iface.set_voltage( m, traj[ind][1])
	ind += 1
	if ind > len(traj)-1:
	    break

print "Trajectory finished"
feedback = iface.feedback
iface.set_voltage( m, 0 )
