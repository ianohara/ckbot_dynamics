'''
A quick class made to load trajectories onto modules in a safe way
'''
import time
import struct
import json

class trajLoader( object ):

    SET_FMT = '<BBhH'
    GET_FMT = '<BB'
    RET_FMT = '<BBhH'

    MAX_TRAJ_LEN = 100
    MAX_TRAJ_TIME = 30
    END_TRAJ = 999

    TORQUE_MAX = 0.452
    TORQUE_TO_VOLTS = 300/TORQUE_MAX

    def __init__( self, module_iface, control_file, module_map ):
        '''
        module_map: list { module_id .... }
        '''
        self.mface = module_iface
        self.mmap = module_map
        self.trajectory = None
        self.load_controls( control_file )

    def load_controls( self, control_file ):
        cdat = json.load(open(control_file, 'r'))["controls"]
        if len(cdat) == 0:
            print "Warning: control data is empty"
        if len(cdat) > self.MAX_TRAJ_LEN:
            print "Warning: control data longer than MAX_TRAJ_LEN"
        self.trajectory = []
        for ctrl in cdat:
            ts = int(ctrl['end_time']*1000) # Convert to ms
            ind = ctrl['start_state_index']
            for m_id, mctrl in zip(self.mmap, ctrl['control']):
                # Note our torque is the opposite of the sim torque
                cmd = int(self.TORQUE_TO_VOLTS*mctrl)
                self.trajectory.append( ( m_id, ind, cmd, ts ))
        # Append end trajectory command to trajectory
        for m_id, mctrl in zip(self.mmap, ctrl['control']):
            self.trajectory.append( ( m_id, ind+1, self.END_TRAJ, 0 ))
        print "Controls successfully set"

    def write_trajectory( self ):
        for ctrl in self.trajectory:
            self.set_cmd_sync( *ctrl )
        # Perform second pass
        for ctrl in self.trajectory:
            cmds = self.get_cmd( *ctrl[:2] )
            if cmds != ctrl:
                print "Error in trajectory"
                return False
        print "Controls written"
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
        get command in trajectory
        m_id -- module id
        ind -- index of command
        '''
        if ind > self.MAX_TRAJ_LEN:
            print "Trajectory longer than MAX_TRAJ_LEN"
            return
        buf = (m_id, ind )
        pkt = '1' + self.mface._encode_data( self.GET_FMT, *buf )
        self.write(pkt)
        t0 = time.time()
        dat = ''
        while True:
            if time.time()-t0 > tout:
                print "Read timed out"
                return
            bt = m.ser.read()
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
    from moduleIface import moduleIface
    m = moduleIface( '/dev/ttyUSB1' )
    mmap = [4]
    control_file = 'tests/control_test.txt'
    tl = trajLoader(m,control_file,mmap)
    tl.write_trajectory()

    '''
    m_id = 3
    cmd = (0,0,0)
    print "m_id: %d, cmd: %s" % (m_id, repr(cmd))

    tl.set_cmd_sync(3, *cmd)
    '''
