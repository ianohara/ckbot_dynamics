from struct import pack, unpack
import moduleIface
from time import time as now, sleep 
from math import pi
from random import random
import json

m = moduleIface.moduleIface('/dev/ttyUSB0')

ALL = 0
HB = 2
NONE = 1
def can_pass( m_id, passthis ):
    pass_pkt = 'w' + pack('BBB', 4, m_id, passthis).encode('hex').upper() + '\r'
    print repr(pass_pkt)
    m.ser.write(pass_pkt)



def set_voltage( vol ):
    for i in xrange(10):   
        m.set_voltage(19,vol)
        sleep(random()/10.0)

def set_zero():
    for i in xrange(10):   
        m.set_voltage(19,0)
        sleep(random()/10.0)


def run_test( test_name, test_time ):

    pkt = '000'
    stop = 'f' + pack('B', len(pkt)).encode('hex').upper() + '100' + '\r'
    start = 'f' + pack('B', len(pkt)).encode('hex').upper() + '0' + pack('B',
    test_time).encode('hex').upper() + '\r'

    print "Running Test: %s" % test_name
    print "Test time is: %f" % test_time

    m.ser.write(start)

    sleep(5.0)
    set_voltage(200)
    sleep(0.1)
    set_zero()

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
    modules = {}
    for pkt in pkts:
        if len(pkt) != 10:
            continue
        vals = unpack('<BHHB', pkt[3:-1])
        m_dat.append(vals)
        m_id = vals[0]
        modules[m_id] = m_id
        pos = vals[1]
        pos_raw = pos 
        if pos_raw > 2**14:
            pos_raw = pos_raw - 2**15
        position = pi*pos_raw/float(2**15)
        #pos = 2.0*pi*vals[1]/2**15
        #if pos > pi:
        #    pos = pos - 2*pi
        time = vals[2]/1000.0
        voltage = float(vals[3])/2**5
        print "ID: %d, pos_raw: %d, Pos: %f, Time: %f, Voltage: %f" % (m_id, pos, position, time, voltage)

    test_dat = {'name': test_name, 'time':test_time, 'data':m_dat,
    'modules':modules}
    print "Saving test data to: %s" % test_name
    f = open('tests/' + test_name, 'w')
    json.dump(test_dat, f)
    f.close()
    print "Test successfully completed"


