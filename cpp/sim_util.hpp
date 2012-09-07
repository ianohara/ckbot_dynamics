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
#ifndef _RUN_SIM_HPP
#define _RUN_SIM_HPP

#include<vector>
#include<iostream>
#include<string>
#include<fstream>
#define _USE_MATH_DEFINES
#include<math.h>

#include<boost/program_options.hpp>

#include<ompl/control/SimpleSetup.h> /* http://ompl.kavrakilab.org/ */
#include<ompl/base/spaces/RealVectorStateSpace.h>
#include<ompl/control/Control.h>
#include<json/json.h> /* http://jsoncpp.sourceforge.net/ */

namespace ob = ompl::base;
namespace oc = ompl::control;

const char DELIMITER = '/';

enum planners {RRT=0, KPIECE1};

struct sim_settings {
    std::string sim_dir;
    std::string desc_path;
    std::string chain_path;
    std::string sim_path;
    std::string result_dir;
    std::string result_path;
    std::string world_path;

    enum planners planner;

    unsigned int min_control_steps;
    unsigned int max_control_steps;
    double dt;
    double min_joint_vel;
    double max_joint_vel;
    double min_torque;
    double max_torque;

    float max_sol_time;
    double custom_angle; /* For use in dynamics_verifier.cpp */
    double custom_rate;  /* For use in dynamics_verifier.cpp */
    double torque;       /* For use in dynamics_verifier.cpp */

    unsigned int debug;
    bool save_full_tree;
    bool collisions;
};

void report_setup(struct sim_settings* psets, std::ostream& out=std::cout);

extern struct sim_settings _DEFAULT_SETS;

bool fill_start_and_goal(const Json::Value& sim_root,
                         std::vector<double>& s0,
                         std::vector<double>& s_fin);

boost::shared_ptr<ckbot::CK_ompl>
load_ckbot_rate_machine(struct sim_settings sets,
                        Json::Value& result_root,
                        std::ostream& out_file=std::cout);

boost::shared_ptr<oc::SimpleSetup>
load_and_run_simulation(boost::shared_ptr<ckbot::CK_ompl> rate_machine_p,
                        std::ostream& out_file,
                        struct sim_settings sets,
                        Json::Value& res_root);

bool save_sol(boost::shared_ptr<oc::SimpleSetup> ss_p,
              struct sim_settings sets,
              Json::Value& res_root);

bool
save_full_tree(boost::shared_ptr<oc::SimpleSetup> ss_p,
               Json::Value& res_root);

bool
run_planner(boost::shared_ptr<oc::SimpleSetup> ss_p,
            struct sim_settings sets, Json::Value& res_root);

ob::PlannerPtr
get_planner(oc::SpaceInformationPtr,
            enum planners);

bool
parse_options(int ac,
              char* av[],
              boost::program_options::variables_map& vm,
              struct sim_settings& sets);

#endif /* _RUN_SIM_HPP */
