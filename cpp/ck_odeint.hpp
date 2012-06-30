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
#include<vector>
#include"ckbot.hpp"

namespace ckbot
{
    class odeConstTorque: public chain_rate
    {
        public:
            odeConstTorque(chain &ch, std::vector< double > T) : chain_rate(ch), T_(T) {};
            void operator() ( const std::vector< double > &s,
                              std::vector< double > &sdot,
                              const double t);
        private:
            std::vector< double > T_;
    };
}
