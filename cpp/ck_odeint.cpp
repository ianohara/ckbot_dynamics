#include<iostream>
#include<vector>
#include<ck_odeint.hpp>
#include<ckbot.hpp>

void
ckbot::odeConstTorque::operator() ( const std::vector< double> &s,
                                    std::vector< double > &sdot,
                                    const double t)
{
   sdot = calc_rate(s, T_); 
}
