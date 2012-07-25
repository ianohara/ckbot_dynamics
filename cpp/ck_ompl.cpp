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

/*
 * This code implements the functions needed by differential (control) planners
 * in OMPL.
 *
 */
#include<ompl/control/ODESolver.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>
#include<ompl/control/Control.h>
#include<ompl/base/spaces/RealVectorStateSpace.h>

namespace oc = ompl::control;

#include"ckbot.hpp"
#include"ck_ompl.hpp"

/*
 * Take a json chain subtree, which is an array of module dictionaries,
 * and turn it into a usable chain object populated by the module defined
 * in the module dictionaries.
 */
boost::shared_ptr<ckbot::CK_ompl>
ckbot::setup_ompl_ckbot(Json::Value& chain_root, std::ostream& out_file)
{
    Json::Value chain_array = chain_root["chain"];
    int num_modules = chain_array.size();
    /* This will never be explicitly freed, just let program end do it. */
    ckbot::module_link* modules_for_chain = new ckbot::module_link[num_modules];
    for (unsigned int link=0; link<num_modules; ++link)
    {
        out_file << " Attempting to fill chain link " << link << " of " << num_modules << std::endl;
        if (! ckbot::fill_module(chain_array[link], &modules_for_chain[link]))
        {
            throw "Error"; /* TODO: Make this more descriptive and useful/correct */
        }
    }
    /* Again, never explictly freed.  Let the program run till death! */
    ckbot::chain *ch = new ckbot::chain(modules_for_chain, num_modules);
    std::cout << "DEBUG, DEEPEST LEVEL: " << ch->describe_self() << std::endl;
    /* Chain rate store a reference to a chain, so this is right memory-wise (right?)
     * de-ref pointer, rate_machine looks for a reference so the dereferenced chain
     * isn't passed as a copy, but instead as a reference.  Think that's right... 
     */
    boost::shared_ptr<ckbot::CK_ompl> rate_machine_p(new ckbot::CK_ompl(*ch));
    return rate_machine_p;
};

bool
ckbot::CK_ompl::stateValidityChecker(const ompl::base::State *s)
{
    return true;
}

void
ckbot::CK_ompl::CKBotODE(const oc::ODESolver::StateType& s,
                         const oc::Control* con,
                         oc::ODESolver::StateType& sdot)
{
    const int N = c.num_links();
    const double *input = con->as<oc::RealVectorControlSpace::ControlType>()->values;
    std::vector<double> T(N);

    for (int i = 0; i < N; i++)
    {
        T[i] = input[i];
    }

    std::vector<double> sdot_vec(2*N);
    sdot_vec = calc_rate(static_cast<std::vector<double> >(s), T);

    for (int i = 0; i < 2*N; i++)
    {
        sdot[i] = sdot_vec[i];
    }
}

/* Currently Unused */
void
ckbot::CKBotODEFunc(const oc::ODESolver::StateType& s,
                    const oc::Control* con,
                    oc::ODESolver::StateType& sdot,
                    ckbot::chain_rate& ch_r)
{
    const int N = ch_r.get_chain().num_links();
    const double *input = con->as<oc::RealVectorControlSpace::ControlType>()->values;
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


