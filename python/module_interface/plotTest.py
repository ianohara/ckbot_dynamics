#!/usr/bin/env python
import json

module_traj = {}

module_param = {3: [18880, 2350, 27216], 
                4: [24868, 8530, 250], 
                5: [23152, 6614, 31600]}

test_name = 'tests/debug_test'

test_dat = json.load( open( test_name, 'r' ))
dat = test_dat['data']
test_time = test_dat['time']
test_name = test_dat['name']
modules = test_dat['modules']
if test_dat.has_key('date'):
    print "Test date: %s" % test_dat['date']

for k,v in module_param.items():
    offs = ((2**15/4-v[1])+(2**15*3/4-v[0]))/2
    module_param[k].append(offs)

for dat_stub in dat:
    module_id = dat_stub[0]
    pos_raw = dat_stub[1]
    if pos_raw > 2**14:
        pos_raw = pos_raw - 2**15
 
    max_tics = module_param[module_id][1]
    min_tics = module_param[module_id][0] - 2**15
    pos = -pi/2 + (pi/2--pi/2)/(max_tics-min_tics)*(pos_raw-min_tics)

    time = dat_stub[2]
    if time < 0:
        continue
    #if time > test_time*1000:
    #    break

    voltage = dat_stub[3]
    if not module_traj.has_key(module_id):
        module_traj[module_id] = [] 
    module_traj[module_id].append([time, pos])

for module in modules.values():
    t_p = numpy.array(module_traj[module])
    plot(t_p[...,0],
         t_p[...,1], 'o')
    
