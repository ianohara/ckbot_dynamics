#!/usr/bin/env python
'''
A quick testing class, just pass it an input trajectory it will run it
and return the actual position trajectory

trajectory = [[t0, inp0],
          [t1, inp1]]
'''
import moduleIface as mI
from time import time as now, sleep
import numpy as np

class trajTester( object ):

    def __init__( self, modules, ser='/dev/ttyUSB0' ):
        self.iface = mI.moduleIface( ser )
        self.modules = modules
        self.feedbackrate = 333
        self.feedback = None
        self.trajectory = None

    def setTrajectory( self, trajectory ):
        self.trajectory = np.array( trajectory )

    def setFeedbackRate( self, feedbackrate ):
        self.feedbackrate = feedbackrate

    def runTrajectory( self ):
        if self.trajectory is None:
            print "Trajectory is Empty, use setTrajectory"
            return

        # Set parameters
        for m in self.modules:
            print "Setting up parameters"
            self.iface.set_param_sync( m, 'fdbkRate', self.feedbackrate )
            self.iface.set_pos( m, 0 )

        sleep( 1.0 )

        # Execute
        self.iface.flush()
        ind = 0
        t0 = now()
        while True:
            self.iface.update()
            curtime = now()
            if curtime - t0 > self.trajectory[ind][0]:
            for m in self.modules:
                self.iface.set_voltage( m, self.trajectory[ind][1] )
            ind += 1
            if ind > len(self.trajectory)-1:
                break

        print "Trajectory Executed"

        feedback = self.iface.feedback

        for m in self.modules:
            self.iface.set_voltage( m, 0 )

        pos = []
        for fdbk in feedback:
            pos.append([fdbk.timestamp-t0, fdbk.pos])

        self.feedback = np.array(pos)

    def plot( self ):
        if self.feedback is None:
            print "No feedback found"
            return
        plot( pos[...,0], pos[...,1], '-ro' )







