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

class Vis( object ):
    def __init__(self, file=None):
        with open(file, 'r') as f:
            self._tree = json.loads(f.read())["tree"]
        self._nodes = self._tree["states"]
        self._edges = self._tree["connections"]

        """ TODO: Add more assertions here to check the input. """
        assert all([len(self._nodes[0]) == len(n) for n in self._nodes]), 'Length of all state vectors (nodes) should be equal.'
        assert len(self._nodes) == len(self._edges), 'Each node needs a description of its edges, even if an empty description.'

    def _raw_json(self):
        print json.dumps(self._tree, sort_keys=True, indent=4)


