#!/usr/bin/python

from math import sqrt, pi
from matplotlib import pyplot as pp
import numpy, sys, json

sys.path.append("..")

from Energy import ETree

e = ETree(file='../../cpp/tests/single_module_tests/theory_period/results/energy.txt')

chain = e._ETree__results["chain"]
r_im1 = numpy.array(chain[0]["r_im1"])

mod_cm = sqrt(numpy.dot(r_im1.T,r_im1)[0][0])
th_tau = 2*pi*sqrt(mod_cm/9.81)

j_angles = [s[0] for s in e.states]
dt = e.time[1] - e.time[0] # Assumes d(dt)/dt = 0

fourier = numpy.fft.fft(j_angles)
freq = numpy.fft.fftfreq(len(e.time), d=dt)

""" Should be a nice sine/cosine wave with equal amplitude peaks and
"   a constant period
"""
pp.figure(1)
pp.subplot(311)
pp.hold(True)
pp.grid(True)
pp.title('Single Module Joint Angle (q0 = %f)' % e.states[0][0])
pp.plot(e.time, j_angles, linewidth=2.0)
pp.legend(('Joint Angle'))

""" Peak of first harmonic in fourier transform should match 2*pi*sqrt(L/g)"""
pp.subplot(312)
pp.title('Fourier decomposition VS Theoretical period')
pp.hold(True)
pp.grid(True)
pp.plot(freq, fourier, linewidth=2.0)
pp.plot([1.0/th_tau, 1.0/th_tau], [0, max(fourier)], 'k', linewidth=2.0)
pp.axis([0, 2.0/th_tau, 0, max(fourier)])
pp.legend(('Fourier', 'Theoretical Natural Freq'))

""" Plot energy over time - should be constant """
pp.subplot(313)
pp.hold(True)
pp.grid(True)
pp.title('Energy over time for 1 undamped zero torque module')

pp.plot(e.time, e.energy, linewidth=2.0)
pp.plot(e.time, e.pe, linewidth=2.0)
pp.plot(e.time, e.ke, linewidth=2.0)
pp.legend(('Total', 'PE', 'KE'))
pp.show()
