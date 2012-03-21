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


#include"ckbot.hpp"
#include"ck_ompl.hpp"
#include<ompl/control/ODESolver.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>
#include<ompl/control/Control.h>
#include<ompl/base/spaces/RealVectorStateSpace.h>

namespace oc = ompl::control;

bool
ckbot::CK_ompl::stateValidityChecker(const ompl::base::State *s)
{
    return true;
}

void
ckbot::CK_ompl::CKBotODE(const oc::ODESolver::StateType& s, const oc::Control* con, oc::ODESolver::StateType& sdot)
{
    const int N = c.num_links();
    const double *input = con->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    std::vector<double> T(N);
    
    for (int i = 0; i < N; i++)
    {
        /*
        std::cout << "Input " << i << " is " << input[i] << "\n";
        std::cout << "State is: " << s[2*i] << ", " << s[2*i+1] << "\n";
        */
        T[i] = input[i];
    }

    std::vector<double> sdot_vec(2*N);
    sdot_vec = calc_rate(static_cast<std::vector<double> >(s), T);
    
    for (int i = 0; i < 2*N; i++)
    {
        sdot[i] = sdot_vec[i];
    }
}    

void
ckbot::CKBotODEFunc(const oc::ODESolver::StateType& s, const oc::Control* con, oc::ODESolver::StateType& sdot, ckbot::chain_rate& ch_r)
{
    const int N = ch_r.get_chain().num_links();
    const double *input = con->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    std::vector<double> T(N);
    for (int i = 0; i < N; i++)
    {
        T[i] = input[i];
        std::cout << "State is: " << s[2*i] << ", " << s[2*i+1] << "\n";
    }

    std::vector<double> sdot_vec(2*N);
    sdot_vec = ch_r.calc_rate(static_cast<std::vector<double> >(s), T);
    
    for (int i = 0; i < 2*N; i++)
    {
        sdot[i] = sdot_vec[i];
    } 
};


