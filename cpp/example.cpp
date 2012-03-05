#include <eigen3/Eigen/Dense>
#include<vector>
#include<iostream>
#define _USE_MATH_DEFINES
#include<math.h>

#include "ckbot.hpp"

int main(void)
{
    double mod_width = 0.2;
    double mod_head_len = mod_width/2.0;
    double mod_mass = 0.5;
    double damping = 0.5;

    Eigen::Vector3d forward_joint_axis(0.0,0.0,1.0);
    Eigen::Vector3d r_im1(-mod_head_len/2, 0, 0);
    Eigen::Vector3d r_ip1(mod_head_len/2, 0, 0);
    Eigen::Matrix3d I_cm;
    Eigen::Matrix3d R_jts;
    Eigen::Matrix3d init_rotation;
    double m = mod_mass;

    I_cm << 1,0,0,
         0,1,0,
         0,0,1;
    
    R_jts << 1,0,0,
         0,1,0,
         0,0,1;

    init_rotation = ckbot::rotY(M_PI/2); 

    struct ckbot::module_description HT1_first_link = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, init_rotation, m}; 


    struct ckbot::module_description HT1 = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, Eigen::Matrix3d::Identity(), m};

    struct ckbot::module_description HT2 = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, Eigen::Matrix3d::Identity(), m};

    ckbot::module_link ck_first = ckbot::module_link(HT1_first_link);
    ckbot::module_link ck_HT1 = ckbot::module_link(HT1);
    ckbot::module_link ck_HT2 = ckbot::module_link(HT2);

    ckbot::module_link chain_modules[] = {ck_first, ck_HT1, ck_HT1, ck_HT2, ck_HT2};
    int num_modules = 5;

    ckbot::chain ch = ckbot::chain(chain_modules, num_modules);

    ckbot::chain_rate rate_machine(ch);

    std::vector<double> s0(2*num_modules);
    std::fill(s0.begin(), s0.end(), 0.0);

    std::vector<double> T(num_modules);
    std::fill(T.begin(), T.end(), 1.0);

    std::vector<double> qdd(num_modules);
    qdd = rate_machine.calc_rate(s0, T);

    std::cout << "Made it out of the rate_machine...\n";
    std::vector<double>::iterator qdd_it;
    for (qdd_it = qdd.begin(); qdd_it != qdd.end(); qdd_it++)
    {
        std::cout << "Link accels: " << *qdd_it << "\n"; 
    }
    return 0;
}
