#include"ckbot.hpp"
#include<ompl/control/ODESolver.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>

namespace oc = ompl::control;

class CK_ompl: public ckbot::chain_rate
{
    public:
        CK_ompl(ckbot::chain& ch): ckbot::chain_rate(ch) {};

        void
        CKBotODE(const oc::ODESolver::StateType& s, const oc::Control* con, oc::ODESolver::StateType& sdot)
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
};

int main(void)
{
 int a = 1;
}
