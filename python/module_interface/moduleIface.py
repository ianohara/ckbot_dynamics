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
        self.pos = 2.0*pi*pos/2**15
        if self.pos > pi:
            self.pos = self.pos - 2*pi
        self.timestamp = now()

class ModuleIface( object ):

    NETWORK_MANAGEMENT = '000'
    EMERGENCY = '0'
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
        self.buf = ''
        self.requests = []
        self.feedback = []
        self.debug = debug

    def debugOut(msg):
        if self.debug: print "ModuleIface:" + msg

    def close( self ):
        self.debugOut(" close: Closing serial connection.")
        self.ser.close()

    def read( self, tout = 0.01 ):
        '''
        Returns a single valid packet where a packet is defined as:
            'a-z',c for CANBus| len0 len1 | data .... | '\r'
        '''
        t0 = now()
        while True:
            # If timeout return
            if now()-t0 > tout:
                return None
            # Read a single byte
            b = self.ser.read()
            #print repr(b)
            #print repr(self.buf)
            # If no data then try to read again
            if b == '':
                continue
            # If we haven't reached the end of the packet then append
            if b != '\r':
                self.buf += b
                continue
            # Parse the packet
            while len(self.buf) > 0:
                c_ind = find(self.buf, 'c')
                if c_ind == -1:
                    self.buf = ''
                    break
                self.buf = self.buf[c_ind+1:]
                try:
                    length = int(self.buf[:2], 16)
                except:
                    self.buf = ''
                    break
                if len(self.buf)-2 == length:
                    pkt = self.buf[2:]
                    self.buf = ''
                    return pkt

    def flush( self ):
        '''
        while True:
            dat = self.read()
            if dat is None:
            return
        '''
        self.ser.flushInput()
        self.feedback = []
        self.requests = []

    def write( self, pkt ):
        print repr(pkt)
        msg = 'c' + pack('B',len(pkt)).encode('hex').upper() + pkt + '\r'
        print repr(msg)
        self.ser.write(msg)

    def discover( self, timeout=2.0 ):
        '''
        reads data off of interface for timeout time and returns all modules seen
        '''
        modules = set()
        t0 = now()
        while True:
            if now()-t0 > timeout:
                break
            pkt = self.read()

            if pkt is None or pkt[0] != '7':
                continue
            decoded_pkt = self._decode_data( self.PKT_FMT['7'], pkt[1:] )
            modules.add( decoded_pkt[0] )
        return modules

    def _parse_pkt( self, pkt ):
        '''
        Look at first byte of packet to determine msg type then decode
        pkt data
        '''
        print repr( self.PKT_FMT[ pkt[0] ] )
        data = self._decode_data( self.PKT_FMT[pkt[0]], pkt[1:] )
        return Message( pkt[0], data )

    def update( self, tout=0.05 ):
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

    def _decode_data( self, fmt, buf ):
        return unpack( fmt, buf.lower().decode('hex'))

    def _encode_data( self, fmt, *buf ):
        return pack(fmt, *buf).encode('hex').upper()

    def set_param( self, m_id, param, val, perm=False ):
        if not mP.has_key( param ):
            print "paraemeter not in mP"
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

    def start( self, m_id ):
        data = ( m_id, 0, 1 )
        pkt = self.COMMAND + self._encode_data( '<BhB', *data )
        self.write(pkt)

    def stop( self, m_id ):
        data = ( m_id, 0, 2 )
        pkt = self.COMMAND + self._encode_data( '<BhB', *data )
        self.write(pkt)

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

    def calibrate( self, m_id ):
        data = ( m_id, 0, 0, 4 )
        pkt = self.COMMAND + self._encode_data(4*'B', *data )
        self.write(pkt)

