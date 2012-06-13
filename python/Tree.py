"""
* CKBot Kinodynamic Planning with OMPL
* Copyright (C) 2012 Ian O'Hara
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
*(at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

"""
* Tools for visualizing planning trees
"""

import json

class Tree( object ):
    def __init__(self, file=None):
        with open(file, 'r') as f:
            self._tree = json.loads(f.read())["tree"]
        self._nodes = self._tree["states"]
        self._edges = self._tree["connections"]
        """ TODO: Add more assertions here to check the input. """
        assert all([len(self._nodes[0]) == len(n) for n in self._nodes]), 'Length of all state vectors (nodes) should be equal.'
        assert len(self._nodes) == len(self._edges), 'Each node needs a description of its edges, even if an empty description.'
        
        self.dim = len(self._nodes[0])
        assert self.dim % 2 == 0, 'The dimensionality should be even, because two DOF are added per joint.'

    def _raw_json(self):
        print json.dumps(self._tree, sort_keys=True, indent=4)

    def controlFromTo(self, f, t):
        """
        Get the control that brings node f to node t. 

        Arguments:
          f - number of node to start from
          t - number of node to go to from node f

        Returns:
          If these two nodes are connected along the directed edge f to t:
            A list of length self.dimension that brings the system from the state
            at node f to the state at node t
          Otherwise:
            None
        """
        if not self.connected(f,t):
            return None
        return self.getEdge(f, t)['control']

    def getEdge(self, f, t):
        """
        If edge f is directionally connected to t, 
        return the edge dictionary.
        """
        for e in self._edges[f]:
            if e["state"] == t:
                return e
        return None

    def edges(self):
        """
        A generator that returns a tuple of ordered pairs (tuple of tuples!)
        in the form ((x1,x2,x3,...,xn), (y1,y2,y3,...,yn)) where an edge starts
        at the point x_vec = (x1,x2,x3,...,xn) and ends at y_vec = (y1, y2, y3,...,yn)

        NOTE/TODO: This doesn't let us figure out which edge we're getting.  Nice
        for plotting just the edges themselves, but not for also labelling.
        """
        for ind, parent in enumerate(self._nodes):
            for e in self._edges[ind]:
                yield (tuple(parent), tuple(self._nodes[e["state"]]))

    def connected(self, f, t):
        """
        See if node f is connected to t by a directed edge from f to t.
        
        (ie: if t is connected to f along a directed edge from t to f, this
         returns False).

        Arguments:
          f - number of node to start from
          t - number of node to go to from node f

        Returns:
          True if f is connected to t in that direction
          False otherwise
        """
        if any([t == e["state"] for e in self._edges[f]]):
            return True
        else:
            return False
