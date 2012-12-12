#!/usr/bin/python
"""
Script for loading the results of running a trajectory with runTest.py.

This also contains the Analyzer and accompanying classes that are really
useful if you want to do anything with the results of a trajectory.

The script is run in the if __name__ == "__main__" so you can
just import this file and use the Analyzer class (which you
construct with a json that looks like the json stored in our results file)

Run with --help option for guidance!
"""
from math import pi

try:
    from trajLoader import exceedRC710PwmFunc
    DEFAULT_PWM_TO_TORQUE=exceedRC710PwmFunc
except ImportError as e:
    print e
    print "WARNING WARNING WARNING: Could not import trajLoader to get the ExceedRc710 pwm -> torque function."
    print "                         You must supply your own to the ResultMotion constructor."
    DEFAULT_PWM_TO_TORQUE=None

class AbstractMotion(object):
    class State(list):
        def __init__(self, *a, **kw):
            """
            Provides .time, .ang, and .torque attributes which access
            list entry 0, 1, and 2 respectively.
            """
            super(AbstractMotion.State, self).__init__(*a,**kw)
            self.time = self[0]
            self.ang = self[1]
            self.torque = self[2]

    def __init__(self):
        """
        Store a Motion, which consists of M times, corresponding positions,
        and corresponding torque commands for a single module.

        """
        self.times = list()
        self.angs = list()
        self.torques = list()

    def getStates(self):
        """
        Yield the states, in order from 0 -> M, as State lists of form:
           [time, ang, torque]
        This is useful for iterating over all states and filtering
        for some.
        """
        for ti,an,to in zip(self.times,self.angs,self.torques):
            yield AbstractMotion.State([ti, an, to])

class ResultMotion(AbstractMotion):
    def __init__(self, states, calib, pwm_to_torque=DEFAULT_PWM_TO_TORQUE):
        """
        Create a motion object from a list of result states. A result
        state is a length 4 list that looks like:
          [bbid, pos in Q15, time in [ms], pwm out of 300]

        ARGUMENTS:
            states - list of result states, as descibed above.  Every
                     bbid should be equal, because this is a motion
                     for one module.
            calib  - length 3 list describing the module's range
                     of motion in Q15.
                     IE:
                       calib = [min_pos, max_pos, middle_pos]
                     Where min_pos is the Q15 reading when the
                     module is at its most negative angle.
                     Min pos is the Q15 reading when the module
                     is at its most positive angle, and
                     middle_pos is some reading from an angle
                     between min_pos and max_pos.
        """
        assert isinstance(states, list),\
               "states should be a list of result states!"
        assert all([isinstance(s,list) for s in states]),\
               "Each entry in states should be a result state which is a list"
        assert all([len(s)==4 for s in states]),\
               "Each entry in states should be a length 4 result state list"
        assert all([s[0] == states[0][0] for s in states]),\
               "All of the bbids in the states should be equal!  We're storing a motion for one module."
        assert len(calib) == 3,\
               "calib should look like [min_pos, max_pos, middle_pos] (%s)" % (str(calib))
        assert pwm_to_torque, "If the default pwm_to_torque function cannot be loaded, you must supply your own."
        
        super(ResultMotion, self).__init__()

        self._min_tics = calib[0]
        self._max_tics = calib[1]
        self._mid_tics = calib[2]
       

        self._pwm2t = pwm_to_torque

        # Store the result states, but sorted in time order
        self._res_states = sorted(states, key=lambda s: s[2])
        self.bbid = self._res_states[0][0] # Safe because of assertion

        self._time_ms = [s[2] for s in self._res_states]
        self._pos_Q15 = [s[1] for s in self._res_states]
        self._pwm_300 = [s[3] for s in self._res_states]

        self._calcTimes()
        self._calcAngs()
        self._calcTorques()
    
    @classmethod
    def toRadians(cls, p, mint, maxt):
        """
        Convert a Q15 module position reading into radians using the
        mint -> maxt Q15 module range of motion readings.
        """
        # TODO: Check if this is right, I jacked it from posTest
        #       I know Uriah and I botched it like 40 times, so
        #       we should check again.
        if maxt < mint:
            mint = mint-(maxt+1)
            p = p - (maxt+1)
            p = p % 2**15
            maxt = 2**15
        return -pi/2 + (pi/2-(-pi/2))/(maxt - mint)*(p-mint)


    def _calcTimes(self):
        self.times = [t/1000.0 for t in self._time_ms]

    def _calcAngs(self):
        self.angs = [self.toRadians(p, self._min_tics, self._max_tics) for p in self._pos_Q15]

    def _calcTorques(self):
        self.torques = [self._pwm2t(pwm) for pwm in self._pwm_300]

class Analyzer(object):
    def __init__(self, jdata=None):
        """
        Create an object that knows how to analyze parkourbot result
        files.

        NOTEs on Notation:
          1. The data from running trajectories on physical chains comes in
             associated with brain board ID numbers as reference.  The result
             file needs to define the real world ordering of these IDs in the
             physical chain.  This ordering, from base to tip of the chain
             (where base = module initially at origin, tip = end of chain
             opposite the base), makes a lot more sense.

             So, the base->tip ordering is used in this class and the variable
             "mn" is used universally to denote a "module number" in a length
             N chain of modules where:
               mn = 0 -> base module, first module
               mn = N-2 -> 2nd to last module
               mn = N-1 -> tip module, last module

             We get rid of the need for the brain board ID numbers immediately,
             so you should not ever need to use them in the context of the
             public methods of this class.

          2. In the case that the Brain Board ids are mentioned, the
             variable bbid is used.

        ARGUMENTS:
          jdata - json data structure that has *AT LEAST*:
                   1. ['results'] key with results from running
                      runTest.py tucked in it.
                   1a. ['results']['module_params'] defining the min,max
                       and middle raw encoder reading for each module
                       in the chain.
                   1b. ['results']['data']

                   2. ['modules'] key that is a list of Brain Board ids (bbid)
                      in order of module numbers (from base to tip)
        """
        assert jdata,\
               "Constructor needs a json data structure with results inside."
        assert jdata.has_key('results'),\
               "Json data structure needs a 'results' top level key"
        assert jdata['results'].has_key('module_params'),\
               "jdata['results'] dictionary needs a module_params key describing each module's min, max, and middle position reading in Q15"
        assert jdata['results'].has_key('data'),\
               "jdata['results'] dictionary needs to have 'data' key."

        self._jdata = jdata
        self._calibs = self._fillCalibs()

        assert jdata.has_key('modules'),\
               "Json data structure needs a 'modules' key that holds a list of module brain board\n\
                ids in order from base to tip."
        self._bbid_to_mn = jdata['modules']

        # Gather the result trajectories in a list of Motion objects
        # ordered from base to tip
        self._resMotions = list()
        self._fillResultMotions()

    def resultMotions(self):
        """
        Yield the result motions in order from
        base (mn=0) to tip (mn=N)
        """
        for rmot in self._resMotions:
            yield rmot

    def _fillCalibs(self):
        """
        The int bbid keys are converted to strings by the json
        parser, which makes indexing into it with numbers a pain
        because we need to convert them to strings first.
        Get rid of the problem by converting to using integer
        keys.
        """
        td = dict()
        for sbbid, calib in self._jdata['results']['module_params'].items():
            td[int(sbbid)] = calib
        return td

    def _fillResultMotions(self):
        """
        Look at our legal json data and fill in self._resMotions, which
        is a list of the physical motions of each module ordered from
        base to tip.
        """
        for bbid in self._bbid_to_mn:
            calib = self._calibs[bbid]
            states = filter(lambda s: s[0]==bbid, self._jdata['results']['data'])
            mot = ResultMotion(states, calib)
            self._resMotions.append(mot)
                
if __name__ == "__main__":
    """
    Analyze a results file.  run this file with the --help option for details
    on the arguments you can use.
    """

    import argparse, json, matplotlib.pyplot as pp, numpy as np
    from os.path import isfile
    from sys import exit
    ap = argparse.ArgumentParser(description="Script for loading and\
    analyzing the result of running a trajectory with runTest.py")
    ap.add_argument('file',
                    type=str,
                    help="Specifies the result (json) file to load")

    ap.add_argument('--times', '-t',
                    type=float,
                    nargs=2,
                    metavar = ('START', 'END'),
                    default = [None, None],
                    dest='t_range',
                    help="Specify the start and end time of region to plot/analyze")

    def inTimeRange(t, t_st, t_end):
        """
        Return True only if t_st <= t <= t_end
          OR
        if t_st and t_end are None.
        """
        if (t_st <= t) and (t <= t_end):
            return True
        elif t_st is None and t_end is None:
            return True
        else:
            return False

    def usage(msg):
        """
        Print argparse usage message and a message describing
        the problem we came across, then exit.
        """
        ap.print_usage()
        print "-"*40
        print msg
        exit(1)

    args = ap.parse_args() 

    # Check all of the inputs, make sure they're sane...
    if not isfile(args.file):
         usage("File (%s) does not exist!" % args.file)

    with open(args.file) as f:
        try:
            jdata = json.load(f)
        except ValueError as e:
            usage("Error loading json: %s" % str(e))

    # Woot, good to go.  Form the analyzer.
    alz = Analyzer(jdata)
    
    fig = pp.figure(num=1)
    fig.hold(True)
    pp.grid(True)
    pp.subplot(311)
    # PLOT 1: Plot the actual trajectories taken
    legend_strings = list()
    for mn, rmot in enumerate(alz.resultMotions()):
        inRangeSt = filter(lambda st: inTimeRange(st[0], args.t_range[0], args.t_range[1]), rmot.getStates())

        pp.plot([s.time for s in inRangeSt], [s.ang for s in inRangeSt]) 
        legend_strings.append("Module %d (bbid=%d)" % (mn, rmot.bbid))
        
    pp.legend(legend_strings)
    pp.show()
