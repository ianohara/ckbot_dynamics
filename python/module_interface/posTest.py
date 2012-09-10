from struct import pack, unpack
import moduleIface
from time import time as now, sleep 
from math import pi
from random import random

m = moduleIface.moduleIface('/dev/ttyUSB1')

ALL = 0
HB = 2
NONE = 1
def can_pass( m_id, passthis ):
    pass_pkt = 'w' + pack('BBB', 4, m_id, passthis).encode('hex').upper() + '\r'
    print repr(pass_pkt)
    m.ser.write(pass_pkt)

test_time = 1

pkt = '000'
stop = 'f' + pack('B', len(pkt)).encode('hex').upper() + '100' + '\r'
start = 'f' + pack('B', len(pkt)).encode('hex').upper() + '0' + pack('B',
test_time).encode('hex').upper() + '\r'

m.ser.write(start)

cmd = 80

#m.set_voltage(19, cmd) # for 4
#m.set_voltage(38, cmd) # for 5
#m.set_voltage(37, 40) # for 3

#sleep( 0.1 )


#for i in xrange(10):
#    m.set_voltage(19, 0)
#    m.set_voltage(38, 0)
#for i in xrange(10):
#    m.set_voltage(37, 0)
sleep(test_time+1)



msg = ''
pkts = []
while True:
    b = m.ser.read()
    if b == '':
        break
    msg += b
    if b == '\r':
        #print repr(msg)
        pkts.append(msg)
        msg = ''

m_dat = []
for pkt in pkts:
    if len(pkt) != 10:
        continue
    vals = unpack('<BhhB', pkt[3:-1])
    m_dat.append(vals)
    m_id = vals[0]
    pos = vals[1]
    #pos = 2.0*pi*vals[1]/2**15
    #if pos > pi:
    #    pos = pos - 2*pi
    time = vals[2]/1000.0
    voltage = float(vals[3])/2**5
    print "ID: %d, Pos: %d, Time: %f, Voltage: %f" % (m_id, pos, time, voltage)
