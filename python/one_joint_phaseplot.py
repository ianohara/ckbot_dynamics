#!/usr/bin/python

import Tree, sys
from matplotlib import pyplot as pp

# For ipython shortcuttery
if len(sys.argv) < 2:
    file = '../cpp/sims/example2/results/results.txt'
else:
    file = sys.argv[1]

t = Tree.Tree(file=file)

def comps(vec, dims):
    """
    Return the components in vec in the
    order specified by dims.
    """
    return [vec[d] for d in dims]

pp.figure(1)
pp.hold(True)
pp.ylabel('Joint Angle Rate')
pp.xlabel('Joint Angle')
for e in t.edges():
    # First module planning tree projection
    pp.plot([e[0][0], e[1][0]], [e[0][t.dim/2], e[1][t.dim/2]])

pp.show()
