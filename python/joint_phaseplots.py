#!/usr/bin/python

from matplotlib import pyplot as pp, cm
import numpy, time, sys

import Tree

# For ipython shortcuttery
if len(sys.argv) < 2:
    file = '../cpp/sims/example_2bodies/results/results.txt'
else:
    file = sys.argv[1]

t = Tree.Tree(file=file)

def comps(vec, dims):
    """
    Return the components in vec in the
    order specified by dims.
    """
    return [vec[d] for d in dims]

def dist(v1, v2):
    return numpy.sqrt(numpy.dot((v2-v1),(v2-v1)))

def color(v1, v2, m, cmap=cm.jet):
    """
    Return color based on distance between two vectors with m 
    being the max possible distance.
    """
    return cmap(int(float(256)*(dist(v1, v2)/(float(m)))))

pp.figure(1)
pp.subplot(121)
pp.hold(True)
pp.title('First Dof')
pp.ylabel('Joint Angle Rate')
pp.xlabel('Joint Angle')
pp.subplot(122)
pp.hold(True)
pp.title('Second DOF')
pp.ylabel('Joint Angle Rate')
pp.xlabel('Joint Angle')

cm_jet = cm.jet
max_dist = max([dist(n, t.start) for n in t.nodes()])

for e in t.edges():
    # Color by distance from edge endpoints to the goal.  Red is close, blue is far.
    e_col = color(t.goal, e[1], max_dist)
    # First module planning tree projection
    pp.subplot(121)
    pp.plot([e[0][0], e[1][0]], [e[0][t.dim/2], e[1][t.dim/2]], color=e_col)
    pp.subplot(122)
    pp.plot([e[0][1], e[1][1]], [e[0][t.dim/2 + 1], e[1][t.dim/2 + 1]], color=e_col)
    print "%f: %s" % (dist(t.goal, e[1]), e_col)

pp.savefig("images/joint_phaseplots_2dof_%d.png" % int(time.time()), dpi=480)
pp.show()
