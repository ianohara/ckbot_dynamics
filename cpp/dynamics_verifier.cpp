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

#define _USE_MATH_DEFINES
#include<math.h>

#include<boost/program_options.hpp>
#include<boost/filesystem.hpp>
#include<boost/bind.hpp>

/* BLARGG!  This is a ridiculous hack.  OMPL packages odeint with its source, and so
 * defines its own ompl specific namespace for the odeint code.
 */
#include<omplext_odeint/boost/numeric/odeint.hpp>
namespace ode = boost::numeric::omplext_odeint;

#include "ckbot.hpp"
#include "ck_odeint.hpp"
#include "ck_ompl.hpp"
#include "sim_util.hpp"
#include "util.hpp"

const static double EPS = 0.0001;

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

          ("dir", po::value<std::string>(&sets.sim_dir))

          ("time",
            po::value<float>(&sets.max_sol_time),
            "Length of time to simulate for.")

          ("debug", po::value<unsigned int>(&sets.debug)->default_value(0u))

          ("angle",
            po::value<double>(&sets.custom_angle),
            "Initial angle of each module (ie: ignore those specified in sim.txt)")

          ("rate",
            po::value<double>(&sets.custom_rate)->default_value(0.0),
            "Initial angle rate of each module (ie: ignore those specified in sim.txt)")

          ("torque",
           po::value<double>(&sets.torque)->default_value(0.0),
           "Constant torque to apply to all joints")

          ("dt",
           po::value<double>(&sets.dt)->default_value(0.01),
           "Time step for simulator")
        ;

        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
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
    sets.desc_path = sets.sim_dir + desc_path;
    sets.chain_path = sets.sim_dir + chain_path;
    sets.sim_path = sets.sim_dir + sim_path;
    sets.result_dir = sets.sim_dir + result_dir;
    sets.result_path = sets.result_dir + result_path;

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
        return 1;
    }

    /* Fill the start and goal positions from the sim JSON tree */
    std::vector<double> s0(2*num_modules);
    std::vector<double> s_fin(2*num_modules);
    fill_start_and_goal(sim_root, s0, s_fin);
//DEBUG
//DEBUG    if (abs(sets.custom_angle - _DEFAULT_SETS.custom_angle) > EPS)
//DEBUG    {
//DEBUG        for (int i=0; i < num_modules; i++)
//DEBUG        {
//DEBUG            s0[i] = sets.custom_angle;
//DEBUG        }
//DEBUG        std::cout << "Using a custom initial joint angle for the modules ("
//DEBUG                  << sets.custom_angle << ")." << std::endl;
//DEBUG    }
//DEBUG    if (abs(sets.custom_rate - _DEFAULT_SETS.custom_rate) > EPS)
//DEBUG    {
//DEBUG        for (int i=0; i < num_modules; i++)
//DEBUG        {
//DEBUG            s0[num_modules+i] = sets.custom_rate;
//DEBUG        }
//DEBUG        std::cout << "Using a custom initial joint rate for modules ("
//DEBUG                  << sets.custom_rate << ")." << std::endl;
//DEBUG    }
//DEBUG
    /* The top level entry "verifications" in the result_root
     * json will store verification runs.  It is an array of
     * arrays of dictionaries. The outer array contains
     * arrays of verifcation runs, and each verification
     * array contains dictionaries with "time", "state",
     * and "energy" entries.
     */

    Json::Value verifications(Json::arrayValue);
    /* First off, we want to verify that with 0 torques and no damping, 
     * system energy is constant
     */
    {
        /* Make the constant torque vector */
        std::vector<double> T(num_modules);
        std::fill(T.begin(), T.end(), 0.0); // DEBUG.  Making this only set the first.
        T[0] = sets.torque;
        double tstart = 0.0; // DEBUG/TODO: Make this command line or json file setting.
        double tend = 0.1; // DEBUG/TODO: Make this command line or json file setting.

        ckbot::odePulseTorque chain_integrator(rate_machine_p->get_chain(), T, tstart, tend);

        ckbot::chain& ch = chain_integrator.get_chain();
        std::cout << ch.describe_self() << std::endl;

        std::vector<double> s_cur(s0);

        Json::Value this_ver(Json::arrayValue);
        ode::runge_kutta4< std::vector< double > > stepper;
        const double dt = sets.dt;
        const double sim_time = sets.max_sol_time;
        for (double t = 0.0; t < sim_time; t += dt)
        {
            stepper.do_step(chain_integrator, s_cur, t, dt);

            Json::Value this_step(Json::objectValue);
            Json::Value this_state(Json::arrayValue);

            double ke = 0.0;
            double pe = 0.0;

            this_step["time"] = t;
            for (int i=0; i < s_cur.size(); i++)
            {
               this_state.append(s_cur[i]);
            }
            for (int j=0; j < s_cur.size()/2; j++)
            {
                ckbot::module_link m = ch.get_link(j);
                Eigen::Vector3d omega_j = ch.get_angular_velocity(j);
                Eigen::Vector3d cur_vel = ch.get_linear_velocity(j);
                Eigen::Matrix3d R_cur = ch.get_current_R(j);

                double ke_cur = ((0.5)*(omega_j.transpose()*R_cur*m.get_I_cm()*R_cur.transpose()*omega_j))[0] + (0.5)*m.get_mass()*(cur_vel.dot(cur_vel));
                ke += ke_cur;
                double pe_cur = ch.get_link_r_cm(j).dot(Eigen::Vector3d::UnitZ())*m.get_mass()*9.81;
                pe += pe_cur;
            }

            this_step["ke"] = ke;
            this_step["pe"] = pe;
            this_step["energy"] = ke+pe;
            this_step["state"] = this_state;
            this_ver.append(this_step);
        }
        verifications.append(this_ver);
    }
    result_root["verifications"] = verifications;

    /* To file! */
    std::ofstream result_file;
    result_file.open((char*)sets.result_path.c_str());
    result_file << result_root;
    result_file.close();
    
    return 0;
}
