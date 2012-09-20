import serial
import struct

s = serial.Serial('/dev/ttyUSB0')
s.setBaudrate(115200)
s.setTimeout(0.1)

# fmt: 't' | len | cmd | ind ind | volt_h volt_h volt_l volt_l | time_h time_h
# time_l time_l
# pkt_len = 11

cmd = (3, 9,999,1)

print repr(cmd)
set_msg = '0' + struct.pack('<BBhH', *cmd).encode('hex').upper()
set_pkt = 't' + struct.pack('B', len(set_msg)).encode('hex').upper() + set_msg + '\r'
print repr(set_pkt)
s.write(set_pkt)

get_cmd = cmd[:2]
get_msg = '1' + struct.pack('<BB', *get_cmd).encode('hex').upper()
get_pkt = 't' + struct.pack('B', len(get_msg)).encode('hex').upper() + get_msg + '\r'
print repr(get_pkt)
s.write(get_pkt)

dat = ''
while True:
    bt = s.read()
    dat += bt 
    if bt == '\r':
        print repr(dat)
        break
   
ret_cmd = struct.unpack('<BBhH', dat[3:-1])
print repr(ret_cmd)
