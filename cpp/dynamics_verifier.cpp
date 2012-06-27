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

#include<boost/program_options.hpp>
#include<boost/filesystem.hpp>
//#include<boost/numeric/odeint.hpp>

/* BLARGG!  This is a ridiculous hack.  OMPL packages odeint with its source, and so
 * links to it in a funky way.
 */
namespace ode = boost::numeric::omplext_odeint; 

#include "ckbot.hpp"
#include "ck_ompl.hpp"
#include "sim_util.hpp"

namespace oc = ompl::control;
namespace ob = ompl::base;

int
main(int ac, char* av[])
{
    std::string sim_dir("");
    std::string desc_path("description.txt");
    std::string chain_path("chain.txt");
    std::string sim_path("sim.txt");
    std::string result_dir("results/");
    std::string result_path("energy.txt");

    struct sim_settings sets = _DEFAULT_SETS;
    boost::program_options::variables_map vm;

    try
    {
        namespace po = boost::program_options;
        po::options_description desc("Usage:");
        desc.add_options()
          ("help", "Prints this help message...")
          ("dir", po::value<std::string>())
        ;

        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
        }
        if (vm.count("dir"))
        {
            sets.sim_dir = vm["dir"].as<std::string>();
        }
        if (vm.count("debug"))
        {
            sets.debug = 1;
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Exception of unknown type!" << std::endl;
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

    boost::shared_ptr<ckbot::CK_ompl> rate_machine_p;

    Json::Value result_root;
    rate_machine_p = load_ckbot_rate_machine(sets, result_root);

    int num_modules = rate_machine_p->get_chain().num_links();

    /* The start and goal positions are in a separate file. 
     * Hopefully this allows easy re-use of config parts
     */
    /* TODO: This sim_file.open is used wherever we load the start and goal.
     *       Should probably just change fill_start_and_goal to accept
     *       the sim struct and open the sim file instead
     */

    std::ifstream sim_file;
    Json::Value sim_root;
    Json::Reader sim_reader;

    sim_file.open((char*)sets.sim_path.c_str());
    bool sim_parse_success = sim_reader.parse(sim_file, sim_root);
    sim_file.close();

    if (!sim_parse_success)
    {
        std::cerr << "Couldn't parse sim file." << std::endl;
        return boost::shared_ptr<oc::SimpleSetup>();
    }

    /* Fill the start and goal positions from the sim JSON tree */
    std::vector<double> s0(2*num_modules);
    std::vector<double> s_fin(2*num_modules);
    fill_start_and_goal(sim_root, s0, s_fin);

    /* First off, we want to verify that with 0 torques and no damping, 
     * system energy is constant
     */
    {
        ckbot::chain& ch = rate_machine_p->get_chain();
        /* Make sure dampings are all zero */
        for (int i=0; i < ch.num_links(); i++)
        {
            ch.get_link(i).set_damping(0.0);
        }
        
        /* Make the zero torque vector */
        std::vector<double> T(num_modules);
        std::fill(T.begin(), T.end(), 0.0);

        std::vector<double> s_cur(s0);

        ode::runge_kutta4< std::vector< double > > stepper;
        const double dt = 0.01;
        const double sim_time = 1;
        for (double t = 0.0; t < sim_time; t += dt)
        {
            stepper.do_step(boost::bind(&ckbot::CK_ompl::CKBotODEIntSignature, &(*rate_machine_p), _1, _2, _3, T), s_cur, t, dt);
            std::cout << "At: " << t << " we have " << s_cur << std::endl;
        }
    }
}
