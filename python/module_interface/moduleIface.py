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


class moduleIface( object ):

    NETWORK_MANAGEMENT = '000'
    EMERGENCY = '0'
    COMMAND = '1'
    REQUEST_RESPONSE = '5'
    REQUEST = '6'
    HB = '7'

    PKT_FMT = {
	'2' : '<' + 'B' + 4*'h', 
	'5' : '<' + 5*'B' + 2*'h',
	'7' : '<' + 5*'B' + 2*'h'
	}

    def __init__( self, dev='/dev/ttyUSB0' ):
	ser = serial.Serial( dev )
	ser.setBaudrate( 115200 )
	ser.setTimeout( 0.001 )
	ser.flush()
	self.ser = ser
	self.buf = ''

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
		self.buf = self.buf[find(self.buf, 'c')+1:]
		length = int(self.buf[:2], 16)
		if len(self.buf)-2 == length:
		    pkt = self.buf[2:]
		    self.buf = ''
		    return pkt
    
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
	pass	

    def _decode_data( self, fmt, buf ):
	return unpack( fmt, buf.lower().decode('hex'))

    def _encode_data( self, fmt, buf ):
	return pack(fmt, buf).encode('hex').upper()

    def set_voltage( self, val ):
	msg = '120' + pack('<h', val).encode('hex').upper()	
	self.write( msg ) 
	
 
	

	    
	
	    
