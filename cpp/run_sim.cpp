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
#include<boost/program_options.hpp>
#include<boost/filesystem.hpp>

#include<ompl/control/SimpleSetup.h>
#include<ompl/control/planners/kpiece/KPIECE1.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>
#include<ompl/base/spaces/RealVectorStateSpace.h>
#include<ompl/control/Control.h>

#include<json/json.h>

#include "ckbot.hpp"
#include "ck_ompl.hpp"

bool fill_start_and_goal(const Json::Value& sim_root, std::vector<double>& s0, std::vector<double>& s_fin);
bool load_and_run_simulation(std::ostream& out_file, const std::string&, const std::string&, const std::string&);
bool fill_module(const Json::Value&, ckbot::module_link*);
ckbot::CK_ompl setup_ckbot(Json::Value& chain_root);
bool save_sol(ompl::control::SimpleSetup& ss, std::ostream& out_file);

const float SOLUTION_TIME = 5;

int main(int ac, char* av[])
{
    std::string sim_dir("");
    std::string desc_path("description.txt");
    std::string chain_path("chain.txt");
    std::string sim_path("sim.txt");
    std::string result_dir("results/");
    std::string result_path("results.txt");

    /* Parse options and execute options */
    namespace po = boost::program_options;
    try 
    {
        po::options_description desc("Usage:");
        desc.add_options()
            ("dir", po::value<std::string>(), 
               "Set the directory in which the simulation to run exists")
            ("help", "Give non-sensical help.")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            return 1;
        }

        if (vm.count("dir")) 
        {
            sim_dir = vm["dir"].as<std::string>();
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Exception of unknown type!" << std::endl;
    }

    /* Check to make sure the simulation directory and all of its components
     * exist
     */
    if (  (sim_dir.length() == 0) 
       || (! boost::filesystem::is_directory(sim_dir)))
    {
        std::cout << "The simululation directory ('" 
                  << sim_dir 
                  << "') specified by --dir must exist!" << std::endl;   
        return 1;
    }
    /* Make sure the directory path ends with a forward slash */
    if (! (sim_dir.at(sim_dir.length()-1) == '/'))
    {
        sim_dir.push_back('/');
    }

    desc_path = sim_dir + desc_path;
    chain_path = sim_dir + chain_path;
    sim_path = sim_dir + sim_path;
    result_dir = sim_dir + result_dir;
    result_path = result_dir + result_path;
    if (! boost::filesystem::is_regular_file(desc_path)) 
    {
        std::cout << "The description file does not exist. (" 
                  << desc_path 
                  << ")" << std::endl;
        return 1;
    }
    if (! boost::filesystem::is_regular_file(chain_path))
    {
        std::cout << "The chain file does not exist." << std::endl;
        return 1;
    }
    if (! boost::filesystem::is_regular_file(sim_path))
    {
        std::cout << "The simulation file does not exist." << std::endl;
        return 1;
    }

    /* Verify and (if needed) create the result directory */
    if (! boost::filesystem::is_directory(result_dir))
    {
        std::cout << "The result directory doesn't exist yet...creating...";
        try 
        {
            boost::filesystem::create_directory(result_dir);
            std::cout << "Last char is: " 
                      << sim_dir.at(sim_dir.length()-1) << std::endl;
        }
        catch (std::exception& e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Unknown error occured while creating directory" << std::endl;
        }
        std::cout << "Success!" << std::endl;
    }

    bool sim_status = load_and_run_simulation(std::cout, chain_path, sim_path, result_path);
    if (! sim_status)
    {
        std::cout << "Error loading simulation..exiting." << std::endl;
        return 1;
    }
    return 0;
}

/*
 * Take a json chain subtree, which is an array of module dictionaries,
 * and turn it into a usable chain object populated by the module defined
 * in the module dictionaries.
 */
ckbot::CK_ompl 
setup_ckbot(Json::Value& chain_root)
{
    Json::Value chain_array = chain_root["chain"];
    int num_modules = chain_array.size();
    struct ckbot::module_link* modules_for_chain = new struct ckbot::module_link[num_modules];
    for (unsigned int link=0; link<num_modules; ++link)
    {
        if (! fill_module(chain_array[link], &modules_for_chain[link]))
        {
            throw "Error";
        }
    }
    ckbot::chain *ch = new ckbot::chain(modules_for_chain, num_modules);
    ckbot::CK_ompl rate_machine(*ch);
    return rate_machine;
}

/*
 * Load a simulation from files containing json descriptions of the chain, 
 * and start and goal positions.
 * Then run and save the results for later. 
 */
bool
load_and_run_simulation(std::ostream& out_file, const std::string& chain_path, const std::string& sim_path, const std::string& result_path)
{
    /*****
    * Load both the CKBot chain simulator and the start and end goals
    *****/
    std::ifstream chain_file;
    std::ifstream sim_file;
    std::ofstream result_file;
    Json::Value chain_root;
    Json::Reader chain_reader;
    Json::Value sim_root;
    Json::Reader sim_reader;

    result_file.open((char*)result_path.c_str());
    result_file << "{" << std::endl;

    chain_file.open((char*)chain_path.c_str());
    bool parsingSuccessful = chain_reader.parse(chain_file, chain_root);
    chain_file.close();
    if (!parsingSuccessful)
    {
        std::cerr << "Couldn't parse chain file." << std::endl;
        return false;
    }
    
    ckbot::CK_ompl rate_machine = setup_ckbot(chain_root);
    rate_machine.get_chain().describe_self(result_file);
    result_file << "," << std::endl;
    int num_modules = rate_machine.get_chain().num_links();
    ckbot::module_link first_module = rate_machine.get_chain().get_link(0u);

    sim_file.open((char*)sim_path.c_str());
    bool sim_parse_success = sim_reader.parse(sim_file, sim_root);
    sim_file.close();

    if (!sim_parse_success)
    {
        std::cerr << "Couldn't parse sim file." << std::endl;
        return false;
    }

    /* Start State */
    std::vector<double> s0(2*num_modules);
    /* Goal State */
    std::vector<double> s_fin(2*num_modules);

    fill_start_and_goal(sim_root, s0, s_fin);

    /*****
     * Set up OMPL 
     *****/
    /* Make our configuration space and set the bounds on each module's angles */
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2*num_modules));
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(first_module.get_joint_min(), 
                                                             first_module.get_joint_max());

    /* Make our control space, which is one bound direction for each joint (Torques) */
    ompl::control::ControlSpacePtr cspace(new ompl::control::RealVectorControlSpace(space, num_modules));

    ompl::base::RealVectorBounds cbounds(num_modules);
    cbounds.setLow(-first_module.get_torque_max());
    cbounds.setHigh(first_module.get_torque_max());

    cspace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds); // TODO (IMO): Arbitrary for now

    /* 
     * Use OMPL's built in setup mechanism instead of allocating state space information
     * and problem defintion pointers on my own
     */
    ompl::control::SimpleSetup ss(cspace);

    // TODO: Write an actual state validity checker!
    ss.setStateValidityChecker(boost::bind<bool>(&ckbot::CK_ompl::stateValidityChecker, 
                                                 rate_machine, 
                                                 _1));

    /* Setup and get the dynamics of our system into the planner */
    ompl::control::ODEBasicSolver<> odeSolver(ss.getSpaceInformation(), 
                                              boost::bind<void>(&ckbot::CK_ompl::CKBotODE, 
                                                                rate_machine, _1, _2, _3));

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

    /* Allow this range of number of steps in our solution */
    ss.getSpaceInformation()->setMinMaxControlDuration(1, 100);

    ss.setup();
    if (ss.solve(SOLUTION_TIME))
    {
        save_sol(ss, result_file);
    } else {
        result_file << "}" << std::endl;
        result_file.close();
        return false;
    }

    result_file << "}" << std::endl;
    result_file.close();   
    return true;
}

/* 
 * Output solution to file in json format
 */
bool 
save_sol(ompl::control::SimpleSetup& ss, std::ostream& out_file)
{
    const ompl::control::PathControl& sol_path(ss.getSolutionPath());
    unsigned int num_modules = (*(ss.getStateSpace())).as<ompl::base::RealVectorStateSpace>()->getDimension();
    std::vector<double> time(sol_path.getStateCount());
    std::vector<double> dt(sol_path.getStateCount()-1);
    out_file << "\"control\": [" << std::endl;   

    time[0] = 0.0;
    for (unsigned int i=0; i < sol_path.getStateCount(); ++i)
    {
        if (i != 0)
        {
            out_file << "{" << std::endl;
            const ompl::base::RealVectorStateSpace::StateType& s_minus = *sol_path.getState(i-1)->as<ompl::base::RealVectorStateSpace::StateType>();
            const ompl::base::RealVectorStateSpace::StateType& s = *sol_path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();

            dt[i-1] = sol_path.getControlDuration(i-1); // Control Duration to go from state i-1 to i;
            time[i] = time[i-1]+dt[i-1]; // Time at this step
            out_file << "\"start_state_index\":" << i-1 << "," << std::endl;
            out_file << "\"end_state_index\":" << i << "," << std::endl;

            out_file << "\"start_state\": [";
            for (unsigned int j=0; j<num_modules; ++j)
            {
                out_file << s_minus[j];
                if (j+1 < num_modules)
                {
                    out_file << ", ";
                }
            }
            out_file << "]," << std::endl;

            out_file << "\"end_state\": [";
            for (unsigned int j=0; j<num_modules; ++j)
            {
                out_file  << s[j];
                if (j+1 < num_modules)
                {
                    out_file << ", ";
                }
            }
            out_file << "]," << std::endl;
            
            out_file << "\"start_time\":" << time[i-1] << "," << std::endl;
            out_file << "\"end_time\":" << time[i] << "," << std::endl;
            out_file << "\"dt\":" << dt[i-1] << "," << std::endl;
            const double* c = sol_path.getControl(i-1)->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
            out_file << "\"control\": [";
            for (unsigned int j=0; j<num_modules; ++j)
            {
                out_file << " " << c[j];
                if (j+1 < num_modules)
                {
                    out_file << ", ";
                }
            }
            out_file << "]";

            /* End this control dictionary and prepare for another, if needed */
            out_file << "}";
            if (i+1 < sol_path.getStateCount())
            {
                out_file << ",";
            }
            out_file << std::endl;
        }
    }
    out_file << "]" << std::endl;
    return true;
}

bool
fill_module(const Json::Value& json_mod, ckbot::module_link* module)
{
    ckbot::module_description this_module_desc;
    double damping = json_mod["damping"].asDouble();
    double mass = json_mod["mass"].asDouble();
    double joint_max = json_mod["joint_max"].asDouble();
    double joint_min = json_mod["joint_min"].asDouble();
    double torque_max = json_mod["torque_max"].asDouble();

    Eigen::Vector3d forward_joint_axis;
    Eigen::Vector3d r_ip1; 
    Eigen::Vector3d r_im1;
    if ((json_mod["f_jt_axis"].size() != 3)
            || (json_mod["r_im1"].size() != 3)
            || (json_mod["r_ip1"].size() != 3))
    {
        return false;
    }
    for (unsigned int m=0; m<3; ++m)
    {
        /*Need 0u as index to distinguish from operator[] which takes a string*/
        forward_joint_axis(m) = ((json_mod["f_jt_axis"])[m])[0u].asDouble();
        r_im1(m) = ((json_mod["r_im1"])[m])[0u].asDouble();
        r_ip1(m) = ((json_mod["r_ip1"])[m])[0u].asDouble();
    } 

    Eigen::Matrix3d I_cm;
    Eigen::Matrix3d R_jts;
    Eigen::Matrix3d init_rotation;
    /* TODO: Check that the json for these 3 are 3x3 arrays!!! */ 
    for (unsigned int m=0; m < 3; ++m)
    {
        for (unsigned int n=0; n<3; ++n)
        {
            I_cm(m,n) = json_mod["I_cm"][m][n].asDouble();
            R_jts(m,n) = json_mod["R_jts"][m][n].asDouble();
            init_rotation(m,n) = json_mod["init_rotation"][m][n].asDouble();
        }
    }

    this_module_desc.damping = damping;
    this_module_desc.m = mass;
    this_module_desc.joint_max = joint_max;
    this_module_desc.joint_min = joint_min;
    this_module_desc.torque_max = torque_max;
    this_module_desc.forward_joint_axis = forward_joint_axis;
    this_module_desc.r_im1 = r_im1;
    this_module_desc.r_ip1 = r_ip1;
    this_module_desc.I_cm = I_cm;
    this_module_desc.R_jts = R_jts;
    this_module_desc.init_rotation = init_rotation;

    ckbot::module_link this_module(this_module_desc);
    *module = this_module;

    return true;
}

bool
fill_start_and_goal(const Json::Value& sim_root, std::vector<double>& s0, std::vector<double>& s_fin)
{
    if((! sim_root["start"].isArray()) || (! sim_root["goal"].isArray()))
    {
        std::cerr << "In simulation file the start and goal positions must be Json arrays." << std::endl;
        return false;
    }
    if ((sim_root["start"].size() != s0.size()) || (sim_root["goal"].size() != s_fin.size()))
    {
        std::cerr << "The start and goal positions in the sim file must have the same dimension as the chain suggests (ie: #modules*2)." << std::endl;
        return false;
    }

    for (unsigned int i = 0; i < s0.size(); ++i)
    {
       s0[i] = sim_root["start"][i].asDouble();
       s_fin[i] = sim_root["goal"][i].asDouble();
    }
    return true;
}
