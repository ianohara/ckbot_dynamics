#!/usr/env/bin python
from testers import trajTester

modules = [38]
trajT = trajTester( modules, ser='/dev/ttyUSB1' )

cmd = 80
traj = [[0,0],
        [0.2, cmd],
	[0.4, 0],
	[10,  0]]

trajT.setTrajectory( traj )
trajT.runTrajectory()

fdbk = trajT.feedback
traj = trajT.trajectory


figure()
ax1 = subplot(211)
ax1.plot( traj[...,0], traj[...,1], '-ro' )

ax2 = subplot(212, sharex=ax1)
ax2.plot( fdbk[...,0], fdbk[...,1], '-bo' )

 

