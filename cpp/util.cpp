/*
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
*/

#include<iostream>
#include<iomanip>

#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>

namespace util
{

/* The stream operator of Eigen::Vector3d prints
 * the vector vertically.  This prints it horizontally
 */
void
flat_print(const Eigen::Vector3d &v, std::ostream& out)
{
    Eigen::IOFormat CleanFmt(4, 0, ", ", ";", "[", "]");
    out << v.format(CleanFmt) << std::endl;
}

}
