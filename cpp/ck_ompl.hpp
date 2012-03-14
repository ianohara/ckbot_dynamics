#ifndef HPP_CK_OMPL
#define HPP_CK_OMPL

#include<vector>
#include<ompl/control/ODESolver.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>
#include<ompl/control/Control.h>

#include"ckbot.hpp"

namespace ckbot
{
    namespace oc = ompl::control;

    class CK_ompl: public chain_rate
    {
        public:
            CK_ompl(chain& ch): chain_rate(ch) {};
            ~CK_ompl(void) {};
            void CKBotODE(const oc::ODESolver::StateType& s, const oc::Control* con, oc::ODESolver::StateType& sdot);
    };

    void CKBotODEFunc(const oc::ODESolver::StateType& s, const oc::Control* con, oc::ODESolver::StateType& sdot, ckbot::chain_rate& ch_r);

};


#endif
