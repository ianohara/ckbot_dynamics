#!/usr/bin/python

from matplotlib import pyplot as pp, cm
import numpy, sys, json

class ETree( object ):
    def __init__(self, file=None):
        with open(file, 'r') as f:
            self.__results = json.loads(f.read())

        self._verifications = self.__results["verifications"][0]

        self.states = [s['state'] for s in self._verifications]
        self.energy = [s['energy'] for s in self._verifications]
        self.ke = [s['ke'] for s in self._verifications]
        self.pe = [s['pe'] for s in self._verifications]
        self.time = [s['time'] for s in self._verifications]

        assert len(self.states) > 1, 'This makes no sense without states!'
        assert len(self.states[0]) % 2 == 0, 'The number of states must be even (ie: module_count*2)'
        assert all([len(s) == len(self.states) for s in [self.energy, self.ke, self.pe, self.time]]), "Number of states, energy, ke, pe, and time entries in 'verifications' root level dictionary of json result file must be the same."

        self.module_count = len(self.states[0]) / 2

if __name__ == "__main__":
    if len(sys.argv) > 1:
        res_file = sys.argv[1]
    else:
        res_file = "../cpp/sims/dyn_ver_sanity/results/energy.txt"

    print "Running on file: %s" % res_file
    e = ETree(file=res_file)

    pp.figure(1)
    pp.hold(True)
    pp.title('Total Energy Over Time (Module Count = %d)' % e.module_count)

    pp.plot(e.time, e.energy, 'or')
    pp.plot(e.time, e.ke, 'b')
    pp.plot(e.time, e.pe, 'g')

    pp.legend(('Total Energy', 'Kinetic', 'Potential'))
    pp.grid(True)
    pp.show()
