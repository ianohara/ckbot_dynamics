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

    enum planners planner;

    unsigned int min_control_steps;
    unsigned int max_control_steps;
    double dt;
    double min_torque;
    double max_torque;

    float max_sol_time;

    unsigned int debug;
    bool save_full_tree;
};

/* For use both as the default initializer for settings structs
 * and as the default parameter in function prototypes that take
 * settings.  NOTE:  Defaults could be dangerous here, because
 * forgetting to pass sim settings into a function won't halt
 * the sim, but will just run that portion of it with the wrong
 * settings!
 */

struct sim_settings _DEFAULT_SETS = {
        "",
        "description.txt",
        "chain.txt",
        "sim.txt",
        "results/",
        "results.txt",

        RRT,

        1,      /* OMPL min control steps */
        1,      /* OMPL max control steps */
        0.05,   /* OMPL timestep resolution */
        -1.0,   /* Minimum link torque */
        1.0,    /* Max link torque */

        30,     /* Solution search timeout in [s] */
        0,      /* Debugging output? */
        true    /* Save the full planning tree? */
};

bool fill_start_and_goal(const Json::Value& sim_root,
                         std::vector<double>& s0,
                         std::vector<double>& s_fin);
boost::shared_ptr<ckbot::CK_ompl> load_ckbot_rate_machine(struct sim_settings sets,
                                                          std::ostream& out_file=std::cout);
boost::shared_ptr<oc::SimpleSetup> load_simulation(boost::shared_ptr<ckbot::CK_ompl> rate_machine_p,
                                                   std::ostream& out_file,
                                                   struct sim_settings sets);
bool save_sol(boost::shared_ptr<oc::SimpleSetup> ss_p,
              std::ostream& out_file,
              struct sim_settings sets=_DEFAULT_SETS);
bool save_full_tree(boost::shared_ptr<oc::SimpleSetup> ss_p, std::ostream& out_file);
bool run_planner(boost::shared_ptr<oc::SimpleSetup> ss_p, struct sim_settings sets);
ob::PlannerPtr get_planner(oc::SpaceInformationPtr, enum planners);
bool parse_options(int ac, char* av[], boost::program_options::variables_map& vm, struct sim_settings& sets);

#endif /* _RUN_SIM_HPP */
