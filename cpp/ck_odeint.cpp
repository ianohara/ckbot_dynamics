#include<iostream>
#include<vector>

using std::cerr;
using std::cout;
using std::endl;

#include<ck_odeint.hpp>
#include<ckbot.hpp>


void
ckbot::odeConstTorque::operator() ( const std::vector< double> &s,
                                    std::vector< double > &sdot,
                                    const double t)
{
   sdot = calc_rate(s, T_); 
}


ckbot::odePulseTorque::odePulseTorque(ckbot::chain &ch,
                      std::vector< double > Tpulse,
                      double t_start,
                      double t_end) :
    chain_rate(ch), /* Parent constructor */
    Tpulse_(Tpulse),
    Tzero_(ch.num_links()),
    ts_(t_start),
    te_(t_end)
{
    if (ts_ > te_) {
        cerr <<  "odePulseTorque(): t_start must be before t_end!" << endl;
        exit(1);
    }

    if (ch.num_links() != Tpulse_.size()) {
        cerr << "odePulseTorque(): Torque vector must be same length as chain." << endl;
        exit(1);
    }
    std::fill(Tzero_.begin(), Tzero_.end(), 0.0);
}


void
ckbot::odePulseTorque::operator() (const std::vector< double > &s,
                                   std::vector< double > &sdot,
                                   const double t)
{
    if ((t > ts_) && (t < te_)) {
        sdot = calc_rate(s, Tpulse_);
    }
    else
    {
        sdot = calc_rate(s, Tzero_);
    }
}

