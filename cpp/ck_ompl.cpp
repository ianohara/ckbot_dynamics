#include"ckbot.hpp"
#include"ck_ompl.hpp"
#include<ompl/control/ODESolver.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>
#include<ompl/control/Control.h>

namespace oc = ompl::control;

void
ckbot::CK_ompl::CKBotODE(const oc::ODESolver::StateType& s, const oc::Control* con, oc::ODESolver::StateType& sdot)
{
    const int N = c.num_links();
    const double *input = con->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
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

void
ckbot::CKBotODEFunc(const oc::ODESolver::StateType& s, const oc::Control* con, oc::ODESolver::StateType& sdot, ckbot::chain_rate& ch_r)
{
    const int N = ch_r.get_chain().num_links();
    const double *input = con->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    std::vector<double> T(N);
    for (int i = 0; i < N; i++)
    {
        T[i] = input[i];
    }

    std::vector<double> sdot_vec(2*N);
    sdot_vec = ch_r.calc_rate(static_cast<std::vector<double> >(s), T);
    
    for (int i = 0; i < 2*N; i++)
    {
        sdot[i] = sdot_vec[i];
    } 
};
