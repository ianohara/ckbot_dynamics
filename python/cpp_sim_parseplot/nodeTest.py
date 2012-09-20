#!/usr/bin/python

import Tree

t = Tree.Tree(file='../cpp/sims/example2/results/results.txt')
t._raw_json()

print "Node 1 connected to Node 2 (in that direction)? %s" % t.connected(1, 2)
print "Node 2 connected to Node 1 (in that direction)? %s" % t.connected(2, 1)

print "What controls does it take from 1 to 2?\n    %s" % t.controlFromTo(1,2)
print "What controls does it take from 2 to 1?\n    %s" % t.controlFromTo(2,1)

print "Edges are stored in t._edges and are arrays of dictionaries.  You can get a specific edge by t.getEdge(from, to).\n This is the edge from 1 to 2:\n    %s" % t.getEdge(1,2)

