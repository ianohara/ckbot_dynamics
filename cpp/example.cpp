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

#include <eigen3/Eigen/Dense>
#include<vector>
#include<iostream>
#include<fstream>
#define _USE_MATH_DEFINES
#include<math.h>
#include<boost/bind.hpp>

#include<ompl/control/SimpleSetup.h>
#include<ompl/control/planners/kpiece/KPIECE1.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>
#include<ompl/base/spaces/RealVectorStateSpace.h>
#include<ompl/control/Control.h>

#include "ckbot.hpp"
#include "ck_ompl.hpp"

void sim_test_0(void);
void sim_test_1(void);
void sim_test_2(void);

const float SOLUTION_TIME = 1000;

int main(void)
{
    sim_test_0();
    sim_test_1();
    sim_test_2();
    return 0;

}

void 
sim_test_0(void)
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

    struct ckbot::module_description HT1_first_link = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, init_rotation, m, -M_PI/2, M_PI/2, 5.0}; 


    struct ckbot::module_description HT1 = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, Eigen::Matrix3d::Identity(), m, -M_PI/2, M_PI/2, 5.0};

    struct ckbot::module_description HT2 = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, Eigen::Matrix3d::Identity(), m, -M_PI/2, M_PI/2, 5.0};

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

    std::vector<double> sd(2*num_modules);
    sd = rate_machine.calc_rate(s0, T);

    std::cout << "Made it out of the rate_machine...\n";
    std::vector<double>::iterator sd_it;
    for (sd_it = sd.begin(); sd_it != sd.end(); sd_it++)
    {
        std::cout << "Link accels: " << *sd_it << "\n"; 
    }
}

void 
sim_test_1(void)
{
    double damping = 1.0;
    double mass = 0.5;
    Eigen::Vector3d forward_joint_axis(0,0,1);
    Eigen::Vector3d r_im1(-0.100, 0.0,0.0);
    Eigen::Vector3d r_ip1(0.100,0.0,0.0);

    Eigen::Matrix3d I_cm;
    Eigen::Matrix3d R_jts;
    Eigen::Matrix3d init_rotation;

    I_cm = Eigen::Matrix3d::Identity();
    R_jts = Eigen::Matrix3d::Identity();
    init_rotation << 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0;

    struct ckbot::module_description HT1 = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, init_rotation, mass, -M_PI/2.0, M_PI/2.0, 5.0};

    ckbot::module_link test_1 = ckbot::module_link(HT1);

    test_1.describe_self(std::cout);

    ckbot::module_link chain_modules[] = {test_1, test_1};
    int num_modules = 2;

    ckbot::chain ch = ckbot::chain(chain_modules, num_modules);

    ckbot::chain_rate rate_machine(ch);

    std::vector<double> s0(2*num_modules);
    std::fill(s0.begin(), s0.end(), 0.0);

    s0[1] = M_PI/2;

    std::vector<double> T(num_modules);
    std::fill(T.begin(), T.end(), 0.0);

    std::vector<double> sd(num_modules);
    std::fill(sd.begin(), sd.end(), 0.0);

    sd = rate_machine.calc_rate(s0, T);

    std::cout << "For Simulation Test 1: 2 HT1 modules with Link 2 at an IC angle\n";
    std::vector<double>::iterator sd_it;
    for (sd_it = sd.begin(); sd_it != sd.end(); sd_it++)
    {
        std::cout << "Link state rates: " << *sd_it << "\n";
    }
}

void 
sim_test_2(void)
{
    std::ofstream out_file;
    out_file.open("control_test.txt");

    out_file << "----BEGINNING OF SIM 2----" << std::endl;

    /* Define the different Modules used in the chain of modules */
    double damping = 1.0;
    double mass = 0.3;
    Eigen::Vector3d forward_joint_axis(0,0,1);
    Eigen::Vector3d r_im1(-0.060, 0.0,0.0);
    Eigen::Vector3d r_ip1(0.060,0.0,0.0);

    Eigen::Matrix3d I_cm;
    Eigen::Matrix3d R_jts;
    Eigen::Matrix3d init_rotation;

    I_cm = Eigen::Matrix3d::Identity();
    R_jts = Eigen::Matrix3d::Identity();
    init_rotation << 0.0, 0.0, 1.0,
                     0.0, 1.0, 0.0,
                    -1.0, 0.0, 0.0;

    struct ckbot::module_description HT1 = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, init_rotation, mass, -M_PI/2, M_PI/2, 10.0};

    ckbot::module_link test_2 = ckbot::module_link(HT1);

    /* Initialize the chain */
    ckbot::module_link chain_modules[] = {test_2, test_2, test_2, test_2};
    int num_modules = 4;
    chain_modules[2].describe_self(std::cout);
    ckbot::chain ch = ckbot::chain(chain_modules, num_modules);
    
    ch.describe_self(out_file);

    /* Initialize the chain rate calculating object and get it ready to hand off to OMPL */
    ckbot::CK_ompl rate_machine(ch);
    /* Start State */
    std::vector<double> s0(2*num_modules);
    std::fill(s0.begin(), s0.end(), 0.0);
    /* Goal State */
    std::vector<double> s_fin(2*num_modules);
    std::fill(s_fin.begin(), s_fin.end(), 0.0);
    s_fin[0] = M_PI;


    /* Set up OMPL */
    /* Make our configuration space and set the bounds on each module's angles */
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2*num_modules));
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(HT1.joint_min, HT1.joint_max);

    /* Make our control space, which is one bound direction for each joint (Torques) */
    ompl::control::ControlSpacePtr cspace(new ompl::control::RealVectorControlSpace(space, num_modules));

    ompl::base::RealVectorBounds cbounds(num_modules);
    cbounds.setLow(-HT1.torque_max);
    cbounds.setHigh(HT1.torque_max);

    cspace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds); // TODO (IMO): Arbitrary for now

    /* 
     * Use OMPL's built in setup mechanism instead of allocating state space information
     * and problem defintion pointers on my own
     */
    ompl::control::SimpleSetup ss(cspace);

    // TODO: Write an actual state validity checker!
    ss.setStateValidityChecker(boost::bind<bool>(&ckbot::CK_ompl::stateValidityChecker, rate_machine, _1));

    /* Setup and get the dynamics of our system into the planner */
    ompl::control::ODEBasicSolver<> odeSolver (ss.getSpaceInformation(), boost::bind<void>(&ckbot::CK_ompl::CKBotODE, rate_machine, _1, _2, _3));
    ss.setStatePropagator(odeSolver.getStatePropagator());

    /* Define the start and end configurations */
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(ss.getSpaceInformation());
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(ss.getSpaceInformation());
    for (int i = 0; i < 2*num_modules; ++i)
    {
        start[i] = s0[i];
        goal[i] = s_fin[i];
    }

    ss.setStartAndGoalStates(start, goal);

    ompl::base::PlannerPtr planner(new ompl::control::KPIECE1(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    /* Allow more than 10 steps in our solution */
    ss.getSpaceInformation()->setMinMaxControlDuration(1, 100);

    ss.setup();
    out_file << "---Simulation Info---" << std::endl;
    out_file << "--The starting point is: " << start << std::endl;
    out_file << "--The goal point is: ";
    ss.getGoal()->print(out_file);
    out_file << std::endl;


    /* Try to find a solution */
    if(ss.solve(SOLUTION_TIME))
    {
        out_file << "---Simulation Solution---" << std::endl;
        /* Output solution to file */
        const ompl::control::PathControl& sol_path(ss.getSolutionPath());

        std::vector<double> time(sol_path.getStateCount());
        time[0] = 0.0;
        out_file << "--Control Inputs--" << std::endl;
        std::vector<double> dt(sol_path.getStateCount()-1);
        for (unsigned int i=0; i < sol_path.getStateCount(); ++i)
        {
            if (i != 0)
            {
                const ompl::base::RealVectorStateSpace::StateType& s_minus = *sol_path.getState(i-1)->as<ompl::base::RealVectorStateSpace::StateType>();
                const ompl::base::RealVectorStateSpace::StateType& s = *sol_path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
                dt[i-1] = sol_path.getControlDuration(i-1); // Control Duration to go from state i-1 to i;
                time[i] = time[i-1]+dt[i-1]; // Time at this step
                out_file << "-Step from " << i-1 << " to " << i << std::endl;
                out_file << "State from [";
                
                for (unsigned int j=0; j<num_modules; ++j)
                {
                    out_file << s_minus[j] << ", ";
                }
                out_file << "] to [";
                for (unsigned int j=0; j<num_modules; ++j)
                {
                    out_file  << s[j] << ", ";
                }
                out_file << "]" << std::endl;

                out_file << "Control from " << i-1 << " to " << i << ":" << " from: " << time[i-1] << "[s] to " << time[i] << "[s] by dt: " << dt[i-1] << "[s]" << std::endl << "\t";
                const double* c = sol_path.getControl(i-1)->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
                for (unsigned int j=0; j<num_modules; ++j)
                {
                    out_file << " " << c[j] << ",";
                }
                out_file << std::endl;
            }
        }
        
    }
    out_file.close();
}
