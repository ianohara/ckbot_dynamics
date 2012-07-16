#!/usr/env/bin python

import moduleIface as mI
from time import time as now, sleep

feedback = []
iface = mI.moduleIface('/dev/ttyUSB1')

modules = [38]

for m in modules:
    print 'Setting param'
    iface.set_param_sync( m, 'fdbkRate', 333 )
    iface.set_pos( m, 0.0 )

# ['dt', 'voltage']
cmd = 80 
traj = [[0   ,  0],
	[0.2,  cmd],
	[0.4,   0],
	[10.0,    0]]

sleep( 1.0 )

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
iface.set_param_sync( m, 'fdbkRate', 0 )
pos = []
for fdbk in feedback:
    pos.append([fdbk.timestamp-t0, fdbk.pos])

traj = numpy.array(traj)
pos = numpy.array(pos)

'''
plot( pos[...,0], pos[...,1], '-ro', 
     traj[...,0],traj[...,1], '-bo' )
'''
plot( pos[...,0], pos[...,1], '-bo' )

