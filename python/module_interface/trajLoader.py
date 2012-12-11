'''
A class made to load trajectories onto modules in a safe way
'''
import time
import struct
import json
from math import pi

DEF_TORQUE_MAX = 0.250
DEF_PWM_COM_MAX = 250
TORQUE_TO_VOLTS = 300/DEF_TORQUE_MAX
_DEFAULT_TORQUE_FUNC = lambda T: int(TORQUE_TO_VOLTS*T)

def sign(v):
    return -1.0 if v < 0.0 else 1.0

def exceedRC710TorqueFunc(T,
                          V_bat=8.0, kv=1022.26, R=0.22,
                          pwm_max=300, pwm_deadzone=25):
    """
    Returns the PWM command (out of 300) to send to Matt's motor controller
    to obtain the torque T (at stall).

    ARGUMENTS:
        T - Torque in [Nm] we want to achieve
        v_bat - Battery voltage (8.0 default is about full charged 2C lipo)
        kv - K_v of the motor in [V/rpm] (Default is from MotorData.xlsx
             in repo:
             personal/picolli/Brushless_Module/Software/Anti_Cogging/ (rev 421))
        R - Resistance of motor coil (default from same spreadsheet)
        pwm_max - Maximum Possible PWM value.
        pwm_deadzone - number of pwm ticks (out of pwm_max) that are dead
                       on the low end

    RETURNS:
        Pwm (out of 300) to command to get desired Torque
    """
    kv_si = kv*(2*pi)/60 # [(rev/min)/v] -> [(rad/s)/v]
    ke_si = 1/kv_si # [Nm/A]

    I = T/ke_si
    V_app = I*R

    pwmCom = int(
                  sign(V_app)*
                    (
                      pwm_max*(abs(float(V_app))/V_bat) + pwm_deadzone
                    )
                )
    assert pwmCom <= pwm_max, """exceedRC710TorqueFunc: pwmCom shouldn't be greater
    than pwm_max (pwmCom=%d)""" % pwmCom
    return pwmCom

def exceedRC710PwmFunc(pwm,
                       V_bat=8.0, kv=1022.26, R=0.22,
                       pwm_max=300, pwm_deadzone=25):
    """
    Returns the commanded Torque as a result of commanding pwm to one of Matt's
    motor controllers controlling an exceed RC 710 motor.
    """
    kv_si = kv*(2*pi)/60 # [(rev/min)/v] -> [(rad/s)/v]
    ke_si = 1/kv_si # [Nm/A]
    if (abs(pwm) < pwm_deadzone):
        return 0.0
    V_app = float(
                   sign(pwm)*
                     (
                       V_bat*(abs(pwm)-pwm_deadzone)/float(pwm_max)
                     )
                 )
    I = V_app/float(R)
    T = I*ke_si 

    return T 

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
                 torque_max=DEF_TORQUE_MAX,
                 pwm_max=DEF_PWM_COM_MAX,
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
        self.torque_max = torque_max
        self.pwm_max = pwm_max

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
                assert mctrl <= self.torque_max, """Before converting to pwm command, 
                asking for torque higher than torque max
                (ask: %2.2f, max: %2.2f)""" % (mctrl, self.torque_max)
                cmd = self.torque_to_pwm_command(mctrl)
                assert cmd <= self.pwm_max, """After converting torque to pwm,
                the resulting pwm is too high. 
                (conversion: %2.2f [Nm] -> %d/300 [pwm] max: %d [pwm])""" % (mctrl, cmd, self.pwm_max)

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
            if ctrl[0] == 3:
                print "SKIPPING 3"
                continue
            cmds = self.get_cmd( *ctrl[:2] )
            if cmds != ctrl:
                print "TrajLoader.write_trajecotory: Error in trajectory"
                return False
        self.debugOut("write_trajectory: Controls written and confirmed.")
        return True

    def write( self, pkt):
        msg = 't' + self.mface._encode_data('B', len(pkt)) + pkt + '\r'
        self.debugOut("write: packet -> '%s'" % msg.encode('hex').upper())
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
                return False
        if ind > self.MAX_TRAJ_LEN:
            print "Trajectory longer than MAX_TRAJ_LEN"
            return False
        if ts > self.MAX_TRAJ_TIME*1000:
            print "Trajectory longer than MAX_TRAJ_TIME"
            return False
        buf = (m_id, ind, cmd, ts )
        pkt = '0' + self.mface._encode_data( self.SET_FMT, *buf )
        self.write(pkt)
        return True

    def get_cmd( self, m_id, ind, tout=0.1, retries=3):
        '''
        get command in trajectory by polling a module
        m_id -- module id
        ind  -- index of command
        '''
        if ind > self.MAX_TRAJ_LEN:
            print "Trajectory longer than MAX_TRAJ_LEN"
            return
        self.debugOut("get_cmd: Asking %d for traj command at index %d" % (m_id, ind))
        buf = (m_id, ind)
        pkt = '1' + self.mface._encode_data( self.GET_FMT, *buf )
        self.write(pkt)
        ret_pkt = ''
        tries = 0
        while not ret_pkt and tries <= retries:
            ret_pkt = self.mface.read(msg_type='t', tout=tout)
            tries+=1
        if not ret_pkt:
            self.debugOut("get_cmd: Read returned empty failed")
            return False
        ret_cmd = struct.unpack( self.RET_FMT, ret_pkt)
        self.debugOut("get_cmd: Unpacked packet (m_id, ind, cmd, ts) = (%d, %d, %d, %d)" % ret_cmd)
        return ret_cmd

    def set_cmd_sync( self, m_id, ind, cmd, ts, retries=2):
        '''
        set command and then get it to check that its valid
        retry a number of times, just in case
        '''
        self.debugOut(
            "set_cmd_sync: Attempting to write packet (id, ind, cmd, t_start) = (%d, %d, %d, %d)" % (
                                                               m_id, ind, cmd, ts)
                     )

        for i in xrange(0, retries):
            self.set_cmd( m_id, ind, cmd, ts )
            ret_cmd = self.get_cmd( m_id, ind )
            if ret_cmd is not None:
                self.debugOut("set_cmd_sync: Write verify response: %s" % repr(ret_cmd))
                if ret_cmd == ( m_id, ind, cmd, ts ):
                    self.debugOut("set_cmd_sync: ...success")
                    return True
                else:
                    self.debugOut("set_cmd_sync: ...failure (attempt %d of %d)" % (i, retries))
            else:
                self.debugOut("set_cmd_sync: No response when trying to verify write (attempt %d of %d)" % (
                                      i, 
                                      retries))
        return False

    def debugOut(self, msg):
        """
        Print a line if debugging is on.
        """
        if self.debug: print msg

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
