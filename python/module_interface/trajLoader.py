'''
A class made to load trajectories onto modules in a safe way
'''
import time
import struct
import json

TORQUE_MAX = 0.452
TORQUE_TO_VOLTS = 300/TORQUE_MAX
_DEFAULT_TORQUE_FUNC = lambda T: int(TORQUE_TO_VOLTS*T)

class TrajLoader( object ):

    SET_FMT = '<BBhH'
    GET_FMT = '<BB'
    RET_FMT = '<BBhH'

    MAX_TRAJ_LEN = 100
    MAX_TRAJ_TIME = 30
    END_TRAJ = 999

    def __init__(self,
                 module_iface=None,  # Module interface for communication
                 control_json=None,  # Json dictionary with "controls" key
                 module_map=None, # List of modules in order from base to tip
                 torque_func=_DEFAULT_TORQUE_FUNC,
                 debug=False
                 ):
        """
        TrajLoader reads in a json file containing a "control" structure (which
        the c++ planner outputs), and loads the specified torque trajectories
        onto each of the physical modules in a chain.  The loaded torques are
        written and then verified.  This does *NOT* run a trajectory, and
        contains no mechanism for actually doing so.

        ARGUMENTS:
          module_iface - A module interface object already setup and ready
                         to talk to modules.
          control_json - json dictionary containing "control"
                         entry with trajectory to load
          module_map - List() - List of modules brain board IDs in
                                order from base to top
          torque_func - Function to convert torques to corresponding
                        pwm commands to write to a motor.  The
                        default is stupid.  Do not use it.
          debug - [False] - Turn on debug output
        """
        if not module_iface:
            raise Exception("You need to supply a module interface")
        self.mface = module_iface

        if not module_map:
            # Try to get it out of the json
            module_map = control_json.get("modules", None)
        if not module_map:
            raise Exception("You need to supply a list of module id (brain board ids) for the modules in order from base to tip")
        self.mmap = module_map

        self.debug = debug
        if self.debug:
            print "Module list from base to tip: ", self.mmap
        if torque_func is _DEFAULT_TORQUE_FUNC:
            print "TrajLoader: Using default torque->pwm conversion function.  BEWARE!  This is probably wrong."
        self.torque_to_pwm_command = torque_func

        self.trajectory = None
        self.load_controls( control_json ) # Will except out on its own via json

    def load_controls( self, jsondat ):
        cdat = jsondat["controls"]
        if len(cdat) == 0:
            print "Warning: control data is empty"
        if len(cdat) > self.MAX_TRAJ_LEN:
            print "Warning: control data longer than MAX_TRAJ_LEN"
        self.trajectory = list()
        for ctrl in cdat:
            ts = int(ctrl['end_time']*1000) # Convert to ms
            ind = ctrl['start_state_index']
            for m_id, mctrl in zip(self.mmap, ctrl['control']):
                # Note our torque is the opposite of the sim torque
                cmd = self.torque_to_pwm_command(mctrl)
                self.trajectory.append( ( m_id, ind, cmd, ts ))
        # Append end trajectory command to trajectory
        for m_id, mctrl in zip(self.mmap, ctrl['control']):
            self.trajectory.append( ( m_id, ind+1, self.END_TRAJ, 0 ))
        print "TrajLoader.load_controls(): Controls successfully loaded from json."

    def write_trajectory( self ):
        for ctrl in self.trajectory:
            self.set_cmd_sync( *ctrl )
        # Perform second pass
        for ctrl in self.trajectory:
            cmds = self.get_cmd( *ctrl[:2] )
            if cmds != ctrl:
                print "TrajLoader.write_trajecotory: Error in trajectory"
                return False
        print "Controls written and confirmed."
        return True

    def write( self, pkt):
        msg = 't' + self.mface._encode_data('B', len(pkt)) + pkt + '\r'
        print repr(msg) # DEBUG
        self.mface.ser.write(msg)

    def set_cmd( self, m_id, ind, cmd, ts ):
        '''
        sets a command in a trajectory
        m_id -- module id
        ind -- index of command
        cmd -- command value from -300 to 300
        ts -- timestamp for cmd in ms
        '''
        if cmd != 999:
            if cmd > 300 or cmd < -300:
                print "command outside of valid range"
                return
        if ind > self.MAX_TRAJ_LEN:
            print "Trajectory longer than MAX_TRAJ_LEN"
            return
        if ts > self.MAX_TRAJ_TIME*1000:
            print "Trajectory longer than MAX_TRAJ_TIME"
            return
        buf = (m_id, ind, cmd, ts )
        pkt = '0' + self.mface._encode_data( self.SET_FMT, *buf )
        self.write(pkt)

    def get_cmd( self, m_id, ind, tout=0.1 ):
        '''
        get command in trajectory by polling a module
        m_id -- module id
        ind  -- index of command
        '''
        if ind > self.MAX_TRAJ_LEN:
            print "Trajectory longer than MAX_TRAJ_LEN"
            return
        buf = (m_id, ind)
        pkt = '1' + self.mface._encode_data( self.GET_FMT, *buf )
        self.write(pkt)
        t0 = time.time()
        dat = ''
        while True:
            if time.time()-t0 > tout:
                print "Read timed out"
                return
            bt = self.mface.ser.read()
            dat += bt
            if bt == '\r':
                break
        ret_cmd = struct.unpack( self.RET_FMT, dat[3:-1] )
        return ret_cmd

    def set_cmd_sync( self, m_id, ind, cmd, ts, retries=3 ):
        '''
        set command and then get it to check that its valid
        retry a number of times, just in case
        '''
        for i in xrange(0, retries):
            self.set_cmd( m_id, ind, cmd, ts )
            ret_cmd = self.get_cmd( m_id, ind )
            if ret_cmd is not None:
                print "cmd: %s" % repr((m_id,ind,cmd,ts))
                print "ret_cmd: %s" % repr(ret_cmd)
                if ret_cmd == ( m_id, ind, cmd, ts ):
                    print "success"
                    break

if __name__ == "__main__":
    from moduleIface import ModuleIface
    import sys
    def usage():
        print "Usage: %s <trajectory file> [<device>]" % sys.argv[0]

    devStr = '/dev/ttyUSB1' # Default works on Linux (if you're lucky)
    controlFile = None
    mmap = [4]
    # The device and trajectory file are specified on cmd line
    if (len(sys.argv) == 3):
        devStr = sys.argv[2]
        controlFile = sys.argv[1]
    # The trajectory is specified, use default device
    elif (len(sys.argv) == 2):
        controlFile = sys.argv[1]
    # D'oh!
    else:
        usage()
        sys.exit(1)

    try:
        m = moduleIface(devStr)
    except Exception as e:
        print "Error making a module interface using device '%s'" % devStr
        print e
        sys.exit(1)
    try:
        tl = trajLoader(m, controlFile, mmap)
    except Exception as e:
        print "Error making a trajLoader with control file '%s'." % sys.argv[2]
        print e

    tl.write_trajectory()
