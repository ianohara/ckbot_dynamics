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

#ifndef HPP_CK_OMPL
#define HPP_CK_OMPL

#include<vector>
#include<ompl/base/GoalState.h>
#include<ompl/base/State.h>
#include<ompl/base/SpaceInformation.h>
#include<ompl/control/ODESolver.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>
#include<ompl/control/Control.h>

#include"ckbot.hpp"
#include"world.hpp"

namespace ckbot
{
    namespace oc = ompl::control;
    namespace ob = ompl::base;

    class CK_ompl: public chain_rate
    {
        public:
            CK_ompl(chain& ch,
                    boost::shared_ptr<World> w=boost::shared_ptr<World>());
            ~CK_ompl(void){};
            void CKBotODE(const oc::ODESolver::StateType& s,
                          const oc::Control* con,
                           oc::ODESolver::StateType& sdot);
           bool stateValidityChecker(const ompl::base::State *s);
           bool setWorld(boost::shared_ptr<World> w);

        private:
           boost::shared_ptr<World> world;
    };

    boost::shared_ptr<ckbot::CK_ompl>
    setup_ompl_ckbot(Json::Value& chain_root,
                     std::ostream& out_file=std::cout);

    void CKBotODEFunc(const oc::ODESolver::StateType& s,
                      const oc::Control* con,
                      oc::ODESolver::StateType& sdot,
                      ckbot::chain_rate& ch_r);

    class EndLocGoalState : public ob::GoalState
    {
        public:
            EndLocGoalState(const ob::SpaceInformationPtr &si, int num_links);
            double distanceGoal(const ob::State *s) const;
        private:
            int num_links_;
    };
};

#endif
