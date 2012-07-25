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

#include<vector>
#include<iostream>
#include<fstream>
#define _USE_MATH_DEFINES
#include<math.h>

#include<boost/bind.hpp> /* www.boost.org */
#include<boost/program_options.hpp>
#include<boost/filesystem.hpp>

#include<Eigen/Dense> /* http://eigen.tuxfamily.org/ */
#include<ompl/control/SimpleSetup.h> /* http://ompl.kavrakilab.org/ */
#include<ompl/control/planners/rrt/RRT.h>
#include<ompl/control/planners/kpiece/KPIECE1.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>
#include<ompl/base/spaces/RealVectorStateSpace.h>
#include<ompl/control/Control.h>
#include<json/json.h> /* http://jsoncpp.sourceforge.net/ */

#include "ckbot.hpp"
#include "ck_ompl.hpp"
#include "sim_util.hpp"

int
main(int ac, char* av[])
{
    std::string sim_dir("");
    std::string desc_path("description.txt");
    std::string chain_path("chain.txt");
    std::string sim_path("sim.txt");
    std::string result_dir("results/");
    std::string result_path("results.txt");

    struct sim_settings sets = _DEFAULT_SETS;
    boost::program_options::variables_map vm;

    if (!parse_options(ac, av, vm, sets))
    {
        return 1;
    }

    /* Check to make sure the simulation directory and all of its components
     * exist
     */
    if (  (sets.sim_dir.length() == 0)
       || (! boost::filesystem::is_directory(sets.sim_dir)))
    {
        std::cerr << "The simululation directory ('"
                  << sets.sim_dir
                  << "') specified by --dir must exist!" << std::endl;
        return 1;
    }

    /* Make sure the directory path ends with a path delimeter */
    if (! (sets.sim_dir.at(sets.sim_dir.length()-1) == DELIMITER))
    {
        sets.sim_dir.push_back(DELIMITER);
    }
    sets.desc_path = sets.sim_dir + sets.desc_path;
    sets.chain_path = sets.sim_dir + sets.chain_path;
    sets.sim_path = sets.sim_dir + sets.sim_path;
    sets.result_dir = sets.sim_dir + sets.result_dir;
    sets.result_path = sets.result_dir + sets.result_path;

    if (! boost::filesystem::is_regular_file(sets.desc_path)) 
    {
        std::cerr << "The description file does not exist. (" 
                  << sets.desc_path 
                  << ")" << std::endl;
        return 1;
    }
    if (! boost::filesystem::is_regular_file(sets.chain_path))
    {
        std::cerr << "The chain file does not exist. ( " <<
           sets.chain_path << ")" << std::endl;
        return 1;
    }
    if (! boost::filesystem::is_regular_file(sets.sim_path))
    {
        std::cerr << "The simulation file does not exist. ( " <<
           sets.sim_path << ")" << std::endl;
        return 1;
    }

    /* Verify and (if needed) create the result directory */
    if (! boost::filesystem::is_directory(sets.result_dir))
    {
        std::cerr << "The result directory doesn't exist yet...creating...";
        try
        {
            boost::filesystem::create_directory(sets.result_dir);
            std::cerr << "Last char is: " 
                      << sets.sim_dir.at(sets.sim_dir.length()-1) << std::endl;
        }
        catch (std::exception& e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
            return 1;
        }
        catch (...)
        {
            std::cerr << "Unknown error occured while creating directory" << std::endl;
            return 1;
        }
        std::cout << "Success!" << std::endl;
    }

    Json::Value result_root;

    boost::shared_ptr<ckbot::CK_ompl> rate_machine_p;
    rate_machine_p = load_ckbot_rate_machine(sets, result_root);

    report_setup(&sets);

    std::cout << rate_machine_p->get_chain().describe_self() << std::endl;

    boost::shared_ptr<oc::SimpleSetup> ss_p;
    ss_p = load_and_run_simulation(rate_machine_p, std::cout, sets, result_root);
    if (!ss_p)
    {
        std::cerr << "Error loading simulation...exiting." << std::endl;
        return 1;
    }
    /* Write the results to file in Json format */
    std::ofstream result_file;
    result_file.open((char*)sets.result_path.c_str());
    result_file << result_root;
    result_file.close();

    return 0;
}


