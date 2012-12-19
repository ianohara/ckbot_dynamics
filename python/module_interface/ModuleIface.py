#!/usr/bin/env python
'''
A quick interface for controlling the modules, there will be a json parser that
will read in json files and send commands to the modules, most likely we will
also have a way to read positions from the modules.

Long term we may have some synchronization method for the modules, so that we
can download each modules motor plan and have them execute the plan in sync.

Test packets:
msg0 = '120' + pack('<h', 0).encode('hex').upper() + '00'
msg50 = '120' + pack('<h', 50).encode('hex').upper() + '00

msg_param_autoZero_off = '620' + pack('B', 17).encode('hex').upper() +7*'00'
msg_calibration = '120' + 2*'00' + pack('B',4).encode('hex').upper()

'''

import serial
from time import time as now, sleep
from struct import pack, unpack
from string import find
from math import pi

class Param( object ):
    def __init__( self, addr, fmt, scl ):
        self.addr = addr
        self.fmt = fmt
        self.scl = scl

PARAM = {
    'id' : Param( 1, 'hh', 1 ),
    'delPWM' : Param( 15, 'hh', 1),
    'fdbkRate' : Param( 16, 'hh', 1 ),
    'autoOff' : Param( 17, 'hh', 1 ),
    }

mP = PARAM

class Message( object ):
    def __init__( self, typ, data ):
        self.type = typ
        self.data = data

class Request( object ):
    def __init__( self, m_id, addr, fmt, tout=0.5 ):
        self.id = m_id
        self.addr = addr
        self.fmt = fmt
        self.data = None
        self.timestamp = None
        self.tout = tout
        self.t0 = now()
        self.dead = False

    def hasData( self ):
        if self.data is None:
            return False
        return True

    def isDead( self ):
        return self.dead

class Feedback( object ):
    def __init__( self, m_id, speed, pwm, current, pos ):
        self.m_id = m_id
        self.speed = speed
        self.pwm = pwm
        self.current = current
        self.pos_raw = pos
        self.timestamp = now()

    def __str__(self):
        return " ".join([str(p) for p in ["Feedback(m_id=",self.m_id,"speed=",self.speed,"pwm=",self.pwm,"current=",self.current,"pos=",self.pos_raw,")"]])

class ModuleIface( object ):
    # Message types known by brain board as of 12/13/2012
    MSG_TYPES = set(['c', # CAN message
                     't', # For setting/getting stored traj from brain board 
                     'f', # "flow control" -> For starting and stopping trajectory 
                     'w'  # "wireless settings" -> used to set CAN forwarding from motro controller
                    ])

    SET_FMT = '<BBhH'
    GET_FMT = '<BB'
    RET_FMT = '<BBhH'

    MAX_TRAJ_LEN = 100
    MAX_TRAJ_TIME = 30
    END_TRAJ = 999

    # For MSG_TYPE 't'
    TRAJ_SET_CMD = '0'
    TRAJ_GET_CMD = '1'

    # For MSG_TYPE 'f'
    TRAJ_START = '0'
    TRAJ_STOP  = '100'

    # NOTE FROM IMO: These are weird values because there is a bug in our bain
    # board code (as of 11/27/2012).  When we look at 'w' messages we look at
    # wrong byte for the data.  So to get a w value of '1' to module id 3 we
    # need to send:
    #  'w040310'
    # instead of:
    #  'w040301'
    # like we wanted.
    # 0x10 = 16 (hence, CAN_NONE)
    # 0x20 = 32 (hence, CAN_HB)
    CAN_ALL = 0
    CAN_HB = 32
    CAN_NONE = 16

    """
      CAN commands that can get passed through to a motor controller
      with the 'c' MSG_TYPE
    """
    COMMAND = '1'
    FEEDBACK = '2'
    REQUEST_RESPONSE = '5'
    REQUEST = '6'
    HB = '7'

    PKT_FMT = {
        FEEDBACK : '<' + 'B' + 4*'h',
        REQUEST_RESPONSE : '<' + 5*'B',
        HB : '<' + 5*'B' + 'h' + 'BB'
    }

    def __init__(self, dev='/dev/ttyUSB0', debug=False):
        ser = serial.Serial( dev )
        ser.setBaudrate( 115200 )
        ser.setTimeout( 0.001 )
        ser.flush()
        self.ser = ser
        self.requests = []
        self.feedback = []
        self.debug = debug

    def debugOut(self, msg):
        if self.debug: print "mIface:" + msg

    def close( self ):
        self.debugOut(" close: Closing serial connection.")
        self.ser.close()

    def read( self, msg_type='c', tout = 0.1 ):
        """
        Returns a single valid CANBus packet where a packet is defined as:
            'a-z', 'c' for CANBus| len0 len1 | data .... | '\r'

        NOTE from IMO: data can contain '\r'!  So we need to use both
             '\r' and length checking to make sure we have a full packet.

        TODO/BUG (IMO): If someone asks to read a 'c' message and there is a
                  't' message before it in the buffer, the 't' message
                  will get clobbered and disappear into the ether!

        """
        if not msg_type in self.MSG_TYPES:
            print "ModuleIface: WARNING: Trying to read unknown message type '%s' (known: %s)" % (msg_type, str(self.MSG_TYPES))
        b = ''
        len_buf = ''
        pkt = ''
        length = None

        S_NOT_IN_PACKET = 0
        S_IN_PACK_NO_LEN = 1
        S_IN_PACK = 2
        S_HAVE_PACKET = 3

        STATE = S_NOT_IN_PACKET
            
        t0 = now()
        self.debugOut("read: Entering state machine with state S_NOT_IN_PACKET.")
        while STATE != S_HAVE_PACKET:
            if now()-t0 > tout:
                self.debugOut("read:  Timing out with pkt='%s'" % str(pkt).encode('hex').upper())
                return None
            b = self.ser.read()
            # If no data then try to read again
            if b == '':
                continue

            if STATE == S_NOT_IN_PACKET:
                """
                Wait until we see a 'msg_type' which signifies the possible
                start of a packet we are interested in.
                """
                if b == msg_type:
                    self.debugOut("read:  Transitioning to S_IN_PACK_NO_LEN")
                    STATE = S_IN_PACK_NO_LEN

            elif STATE == S_IN_PACK_NO_LEN:
                """
                We are in the packet, but we don't know what length it is supposed
                to be.
                """
                #self.debugOut("read: len_buf='%s'" % len_buf)
                len_buf += b
                if len(len_buf) == 2:
                    length = int(len_buf, 16) 
                    self.debugOut("read: Transitioning to S_IN_PACK")
                    STATE = S_IN_PACK

            elif STATE == S_IN_PACK:
                """
                We are in the packet. Look for '\r' which signifies
                possible ends of packets, and when we find them
                make sure the packet length matches the length
                suggested by the packet header.
                
                Also, start over if we see more than length bytes
                without seeing '\r'
                """
                if len(pkt) > length:
                    b = ''
                    pkt = ''
                    len_buf = ''
                    STATE = S_NOT_IN_PACKET
                elif b == '\r' and len(pkt) == length: # Woot!
                    self.debugOut("read: Transitioning to S_HAVE_PACKET")
                    STATE = S_HAVE_PACKET # Exit state
                else:
                    #self.debugOut("read: len(pkt)=%d length=%d pkt='%s'" % (len(pkt),
                    #length, str(pkt).encode('hex')))
                    pkt += b
        return pkt

    def flush( self ):
        """
        Flush the serial buffer.

        NOTE: This is broken on OS X 10.6.8 with the FTDI driver we
              all use (errr...sorry, cannot remember the name of it.
              It's the first result on google for OS X serial driver)
        """
        self.ser.flushInput()
        self.feedback = []
        self.requests = []

    def _decode_data( self, fmt, buf ):
        return unpack( fmt, buf.lower().decode('hex'))

    def _encode_data( self, fmt, *buf ):
        return pack(fmt, *buf).encode('hex').upper()

    def write(self, msg_type, pkt):
        assert msg_type in self.MSG_TYPES,\
               "Unknown message type '%s'.  Choices are: %s" % (msg_type, str(self.MSG_TYPES))
        msg = msg_type + pack('B',len(pkt)).encode('hex').upper() + pkt + '\r'
        self.ser.write(msg)
        self.debugOut("mIface.write: msg='%s'" % msg.replace('\r', '\\r'))
        return msg # For the caller's debugging purposes

    def set_cmd( self, bbid, ind, cmd, ts ):
        '''
        sets a command in a trajectory

        ARGUMENTS:
            bbid -- brain board module id
            ind -- index of command in trajectory
            cmd -- command value from -300 to 300, 999 to "End trajectory"
                   NOTE: I cannot see any reason in the brain board code why 999 is 
                         different than 0! (IMO, 12/13/2012)
            ts -- timestamp for cmd in ms
        '''
        if cmd != 999:
            if cmd > 300 or cmd < -300:
                print "command outside of valid range"
                return False
        if ind > self.MAX_TRAJ_LEN:
            print "Trajectory index longer than MAX_TRAJ_LEN"
            return False
        if ts > self.MAX_TRAJ_TIME*1000:
            print "Trajectory time longer than MAX_TRAJ_TIME"
            return False
        buf = (bbid, ind, cmd, ts )
        pkt = self.TRAJ_SET_CMD + self._encode_data( self.SET_FMT, *buf )
        self.write('t', pkt)
        return True

    def get_cmd( self, bbid, ind, tout=0.1, retries=3):
        '''
        get command in trajectory by polling a module

        ARGUMENTS:
            bbid - brain board module id
            ind  - index of command in trajectory
            tout[=0.1] - Time for a single get to timeout
            retries[=3] - Number of times to retry if we get no response
        RETURNS:
            On success, a packet (bbid, ind, cmd, t_start [ms])
        '''
        if ind > self.MAX_TRAJ_LEN:
            print "get_cmd: Cannot get trajectory index longer than %d (tried to set %d).  Skipping." % (self.MAX_TRAJ_LEN, id)
            return
        self.debugOut("get_cmd: Asking %d for traj command at index %d" % (bbid, ind))
        buf = (bbid, ind)
        pkt = self.TRAJ_GET_CMD + self._encode_data( self.GET_FMT, *buf )
        self.write('t', pkt)
        ret_pkt = ''
        tries = 0
        while not ret_pkt and tries <= retries:
            ret_pkt = self.read(msg_type='t', tout=tout)
            tries+=1
        if not ret_pkt:
            self.debugOut("get_cmd: Read returned empty failed")
            return False
        ret_cmd = unpack( self.RET_FMT, ret_pkt)
        self.debugOut("get_cmd: Unpacked packet (bbid, ind, cmd, ts) = (%d, %d, %d, %d)" % ret_cmd)
        return ret_cmd

    def set_cmd_sync( self, bbid, ind, cmd, ts, retries=2):
        """
        Set command and then get it to check that its valid
        retry a number of times, just in case
        """
        self.debugOut(
            "set_cmd_sync: Attempting to write packet (bbid, ind, cmd, t_start) = (%d, %d, %d, %d)" % (
                                                               bbid, ind, cmd, ts)
                     )

        for i in xrange(0, retries):
            self.set_cmd( bbid, ind, cmd, ts )
            ret_cmd = self.get_cmd( bbid, ind )
            if ret_cmd is not None:
                self.debugOut("set_cmd_sync: Write verify response: %s" % repr(ret_cmd))
                if ret_cmd == ( bbid, ind, cmd, ts ):
                    self.debugOut("set_cmd_sync: ...success")
                    return True
                else:
                    self.debugOut("set_cmd_sync: ...failure (attempt %d of %d)" % (i, retries))
            else:
                self.debugOut("set_cmd_sync: No response when trying to verify write (attempt %d of %d)" % (
                                      i, 
                                      retries))
        return False

    def discover(self, timeout=2.0):
        '''
        reads data off of interface for timeout time and returns all modules seen

        NOTE/TODO/WHY OH WHY!?:  This discovers *motor controller* ids.  There
          is no known way to discover brain board ids....see ping...

        NOTE/TODO: If can_pass(bbid, self.CAN_HB) has not been set for a module,
              this will not find it!
        '''
        modules = set()
        t0 = now()
        while True:
            if now()-t0 > timeout:
                break
            pkt = self.read()
            if pkt is None or pkt[0] != self.HB:
                continue
            decoded_pkt = self._decode_data( self.PKT_FMT[self.HB], pkt[1:] )
            modules.add( decoded_pkt[0] )
        return modules

    def ping(self, bbid, tout=1.0):
       """
       Try to "ping" the brain board with bbid for an ID.  This is
       an existence ping, so it doesn't return any timing info.

       NOTE: As of 12/12/2012 there's no non-hacky way of doing this, so
             I just ask for the module's first trajectory segment and
             see if it responds.

       ARGUMENTS:
           bbid - Brain board ID to ping
       RETURNS:
           True if the board responds
           False otherwise
       """
       if self.get_cmd(bbid, 0,tout=tout, retries=0):
           return True
       else:
           return False

    def scan(self, id_iter=xrange(1,10), tout=0.5):
        """
        Scan a range of brain board ids and see which ones
        respond.

        NOTE: This can take a long time (ie: if len(id_range) == 10
              and tout=1.0 and none of the bbids exist, then this
              will take 10 seconds)

        ARGUMENTS:
           id_iter - An iterable that returns valid
                      brain board ids.
           tout[=0.5] - Timetout for each bbid (in seconds)

        RETURN:
           bbids - a Set of brain board ids that we saw in the range
        """
        bbids = set()
        for bbid in id_iter:
            if self.ping(bbid, tout=tout):
                bbids.add(bbid)
        return bbids

    def _parse_pkt(self, pkt):
        '''
        Look at first byte of packet to determine msg type then decode
        pkt data
        '''
        print repr( self.PKT_FMT[ pkt[0] ] )
        data = self._decode_data( self.PKT_FMT[pkt[0]], pkt[1:] )
        return Message( pkt[0], data )

    def update( self, tout=0.05 ):
        """
        Read for CAN messages from the motor controller and deal with them.

        NOTE from IMO: What happens to requests here when they are fulfilled?
                       Is it assumed that whoever made the request has a
                       reference to it and will check for data?

        NOTE: This uses read() which as of 12/13/2012 squashes messages of
              the wrong type until the buffer runs out, or we find a message
              of the correct type (in this case, the 'c' message for CAN)
              So, be careful!
        """
        t0 = now()
        while True:
            curtime = now()
            if curtime-t0 > tout:
                break
            pkt = self.read()
            if pkt is None:
                continue
            # Handle request responses in a somewhat smart way
            if pkt[0] == self.REQUEST_RESPONSE:
                print "Request Identified: %s" % pkt # DEBUG
                data = self._decode_data( self.PKT_FMT[self.REQUEST_RESPONSE],
                pkt[1:-8] )
                m_id = data[0]
                addr = data[1]
                req_len = len(self.requests)
                cnt = 0
                while cnt < req_len:
                    request = self.requests.pop(0)
                    if request.id == m_id:
                        if request.addr == addr:
                            request.data = self._decode_data( request.fmt,
                            pkt[-8:] )[0]
                            request.timestamp = curtime
                            break
                    self.requests.append( request )
                    cnt += 1
            # Handle feedback
            if pkt[0] == self.FEEDBACK:
                data = self._decode_data( self.PKT_FMT[self.FEEDBACK], pkt[1:]
                )
                self.feedback.append( Feedback( *data ))

            # Get rid of out of timed out requests
            req_len = len(self.requests)
            cnt = 0
            while cnt < req_len:
                if len(self.requests) == 0:
                    break
                request = self.requests.pop(0)
                if curtime - request.t0 > request.tout:
                    request.dead = True
                    continue
                self.requests.append( request )
                cnt += 1

    def set_param( self, m_id, param, val, perm=False ):
        if not mP.has_key( param ):
            print "Parameter not in mP"
            return
        if perm:
            addr = mP[param].addr
        else:
            addr = 256-mP[param].addr
        data = ( m_id, addr, 0, 0, 0,  val )
        pkt = self.REQUEST + self._encode_data( '<'+5*'B'+mP[param].fmt[0], *data )
        self.write( pkt )

    def request_param( self, m_id, param ):
        if not mP.has_key( param ):
            print "param not in mP"
            return
        addr = mP[param].addr
        fmt = mP[param].fmt
        data = ( m_id, 0, addr )
        pkt = self.REQUEST + self._encode_data( '<'+3*'B', *data )
        self.write( pkt )
        request = Request( m_id, addr, fmt )
        self.requests.append( request )
        return request

    def set_param_sync( self, m_id, param, val, perm=False, tout=0.5 ):
        '''
        Set a parameter and then check to make sure that
        the correct value is returned
        '''
        self.set_param( m_id, param, val, perm )
        if perm:
            sleep(2.0)
        req = self.request_param( m_id, param )
        req_val = None
        t0 = now()
        while now()-t0 < tout:
            self.update()
            if not req.hasData():
                continue
            req_val = req.data
            print req_val
            print val
            if req_val == val:
                return True
            self.set_param( m_id, param, val, perm )
            sleep(0.05)
            req = self.request_param( m_id, param )
            sleep(0.1)
        return False

    def set_voltage( self, m_id, val ):
        data = ( m_id, val, 0 )
        pkt = self.COMMAND + self._encode_data( '<BhB', *data )
        self.write(pkt)

    def set_pos( self, m_id, val ):
        val = int(2**15*val/(2.0*pi))
        print "val_raw: %d" % val
        data = ( m_id, val, 3 )
        pkt = self.COMMAND + self._encode_data( '<BhB', *data )
        self.write(pkt)

    def start_traj(self, test_time):
        """
        Send out the message that tells modules to start a trajectory
        and report feedback for a certain length of time.

        ARGUMENTS:
            test_time - Time in [s] to report feedback for (with
                        the staggered reporting of 5 modules tactic)
        """
        test_time = int(test_time)
        if test_time > 255:
            print "mIface.start_traj: Warning, test time in [s] must fit in a Byte (it is %d).  Using 255" % test_time
            test_time = 255
        self.write('f', self.TRAJ_START + self._encode_data('B', test_time))

    def stop_traj(self):
        """
        Send the broadcast message that tells everyone to stop their trajectory.
        This stops both feedback and trajectory running.
        """
        self.write('f', self.TRAJ_STOP)

    def calibrate( self, m_id ):
        data = ( m_id, 0, 0, 4 )
        pkt = self.COMMAND + self._encode_data(4*'B', *data )
        self.write(pkt)

    def can_pass(self, m_id, passthis):
        """
        For brain board id (m_id) set the the filtering function that
        decides what communication from the motor controller gets passed
        through the brain board and to our serial port.

        ARGUMENTS:
          m_id - Numerical brain board ID to set filter on
          passthis - Either self.NONE, self.HB, or self.ALL where
                       self.CAN_NONE - Do not pass anything through
                       self.CAN_HB   - Pass only the heartbeat through
                       self.CAN_ALL  - Pass Everything through (Feedback!)
        """
        if passthis not in (self.CAN_NONE, self.CAN_HB, self.CAN_ALL):
            raise ValueError("passthis must be one of: self.CAN_NONE, self.CAN_HB, self.CAN_ALL")
        pass_pkt = 'w' + pack('<BBB', 4, m_id, passthis).encode('hex').upper() + '\r'
        self.debugOut("can_pass: Setting id=%d to value=%d (packet='%s')"
                        % (m_id, passthis, pass_pkt))
        self.ser.write(pass_pkt)
