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
        if len(cdat) > self.mface.MAX_TRAJ_LEN:
            print "Warning: control data longer than MAX_TRAJ_LEN"
        self.trajectory = list()
        for ctrl in cdat:
            ts = int(ctrl['end_time']*1000) # Convert to ms
            ind = ctrl['start_state_index']
            # self.mmap -> list of brain board ids from base to tip
            # ctrl['control'] => list of torques to apply from base to tip
            for m_id, mctrl in zip(self.mmap, ctrl['control']):
                # Note our torque is the opposite of the sim torque
                # TODO (IMO 12/13/2012): VERIFY THIS!  I cannot remember...
                assert mctrl <= self.torque_max, """Before converting to pwm command, 
                asking for torque higher than torque max
                (ask: %2.2f, max: %2.2f)""" % (mctrl, self.torque_max)
                cmd = self.torque_to_pwm_command(mctrl)
                assert cmd <= self.pwm_max, """After converting torque to pwm,
                the resulting pwm is too high. 
                (conversion: %2.2f [Nm] -> %d/300 [pwm] max: %d [pwm])""" % (mctrl, cmd, self.pwm_max)

                self.trajectory.append( ( m_id, ind, cmd, ts ))
        # Append end trajectory command to trajectory
        # IMO: Why is this needed?  Isn't the whole trajectory initialized to 0 on 
        #      the micro?  Looking in ModuleBrain.c I can't see how 999 does anything
        #      more than set pwm command to 0 (ie: it doesn't stop the trajectory
        #      or anything.)
        for m_id, mctrl in zip(self.mmap, ctrl['control']):
            self.trajectory.append( ( m_id, ind+1, self.mface.END_TRAJ, 0 ))
        print "TrajLoader.load_controls(): Controls successfully loaded from json."

    def write_trajectory( self ):
        for ctrl in self.trajectory:
            self.mface.set_cmd_sync( *ctrl )

        # Perform second pass of verification
        # TODO (IMO): Why do we need this?  Doesn't set_cmd_sync get and confirm?
        for ctrl in self.trajectory:
            cmds = self.mface.get_cmd( *ctrl[:2] )
            if cmds != ctrl:
                print "TrajLoader.write_trajecotory: Error in trajectory"
                return False
        self.debugOut("write_trajectory: Controls written and confirmed.")
        return True

    def write( self, pkt):
        msg =  self.mface.write('t', pkt)
        self.debugOut("write: packet -> '%s'" % msg)

    def debugOut(self, msg):
        """
        Print a line if debugging is on.
        """
        if self.debug: print msg

if __name__ == "__main__":
    from ModuleIface import ModuleIface
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
