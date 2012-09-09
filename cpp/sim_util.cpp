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
#include "world.hpp"

/* For use as the default initializer for settings structs */
struct sim_settings _DEFAULT_SETS = {
        "",
        "description.txt",
        "chain.txt",
        "sim.txt",
        "results/",
        "results.txt",
        "world.txt",

        RRT,

        1,      /* OMPL min control steps */
        1,      /* OMPL max control steps */
        0.05,   /* OMPL timestep resolution */
        100.0,    /* Max joint vel [rad/s] */
        -100.0,   /* Min joint vel [rad/s] */
        -1.0,   /* Minimum link torque */
        1.0,    /* Max link torque */
        0.01,   /* Goal threshhold */

        10,     /* Solution search timeout in [s] */
        1337.0, /* Unused in run_sim.  This is for specifying custom initial 
                 *  joint angle in dynamics_verifier.cpp. Any value over pi
                 *  is nonsense, so the default is just a large value to
                 *  signify that it should be ignored. */
        1337.0, /* Same as above, but for angular rate. */
        0.0,    /* The torque value for dynamics verifier */
        0,      /* Debugging output? */
        true,   /* Save the full planning tree? */
        false,   /* Use collisions? (requires world.txt file) */
};

void
report_setup(struct sim_settings* ps, std::ostream& o)
{
    o << "Running simulation with the following settings: " << std::endl <<
         "  Sim Dir: '" << ps->sim_dir << "'" << std::endl <<
         "  Planner: " << ps->planner << std::endl <<
         "  Min Control Steps: " << ps->min_control_steps << std::endl <<
         "  Max Control Steps: " << ps->max_control_steps << std::endl <<
         "  Goal Threshhold: " << ps->threshhold << std::endl <<
         "  Time Step: " << ps->dt << std::endl <<
         "  Min Torque: " << ps->min_torque << std::endl <<
         "  Max Torque: " << ps->max_torque << std::endl <<
         "  Max Sol Time: " << ps->max_sol_time << std::endl <<
         "  Debug?: " << ps->debug << std::endl <<
         "  Save full tree?: " << ps->save_full_tree << std::endl <<
         "  Use collisions?: " << ps->collisions << std::endl;
}

boost::shared_ptr<ckbot::CK_ompl>
load_ckbot_rate_machine(struct sim_settings sets, Json::Value& res_root, std::ostream& out_file)
{

    std::ifstream chain_file;
    chain_file.open((char*)sets.chain_path.c_str());
    Json::Reader chain_reader;
    Json::Value chain_root;
    bool parsingSuccessful = chain_reader.parse(chain_file, chain_root);
    chain_file.close();
    if (!parsingSuccessful)
    {
        std::cerr << "Couldn't parse chain file." << std::endl;
        return boost::shared_ptr<ckbot::CK_ompl>();
    }

    /*
     * Load the chain and initialize an ODE solver for it from the JSON tree
     * we now have. Also, while we're at it, put the chain into our result
     * JSON object
     */
    boost::shared_ptr<ckbot::CK_ompl> rate_machine_p;
    rate_machine_p = ckbot::setup_ompl_ckbot(chain_root);
    if (!rate_machine_p) 
    {
        return boost::shared_ptr<ckbot::CK_ompl>();
    }
    res_root["chain"] = rate_machine_p->get_chain().describe_self();

    /* Setup the collision world, or leave it as a null pointer
     * to signify that no collision checking should be used
     */
    boost::shared_ptr<World> w;
    w = boost::shared_ptr<World>();
    if (sets.collisions)
    {
        std::ifstream world_file;
        Json::Value world_root;
        Json::Reader world_reader;

        world_file.open((char*)sets.world_path.c_str());
        bool world_parse_success = world_reader.parse(world_file, world_root);
        world_file.close();

        if (!world_parse_success) {
            std::cerr << "Couldn't parse world file." << std::endl;
            return boost::shared_ptr<ckbot::CK_ompl>();
        }
        w = get_world(world_root);
        if (!w) {
            std::cerr << "Couldn't fill the world from world json." << std::endl;
            return boost::shared_ptr<ckbot::CK_ompl>();
        }
        w->describe();
        rate_machine_p->setWorld(w);
    }

    return rate_machine_p;
}

/*
 * Load a simulation from files containing json descriptions of the chain, 
 * and start and goal positions.
 * Then run and save the results for later.
 */
boost::shared_ptr<oc::SimpleSetup>
load_and_run_simulation(boost::shared_ptr<ckbot::CK_ompl> rate_machine_p,
                        std::ostream& out_file,
                        struct sim_settings sets,
                        Json::Value& res_root)
{
    /* For reference when setting up OMPL */
    const int num_modules = rate_machine_p->get_chain().num_links();
    ckbot::chain ch = rate_machine_p->get_chain();
    ckbot::module_link first_module = ch.get_link(0u);

    /*****
    * Load both the CKBot chain simulator and the start and end goals
    * While loading the chain description, start outputing the description
    * to the result json object.
    *****/
    std::ifstream sim_file;
    Json::Value sim_root;
    Json::Reader sim_reader;

    /* The start and goal positions are in a separate file than the chain def.
     * Hopefully this allows easy re-use of config parts
     */
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

    /* Get the start and goal into the json result object */
    Json::Value start_node(Json::arrayValue);
    for (unsigned int i = 0; i < s0.size(); i++)
    {
        start_node.append(s0[i]);
    }
    res_root["start"] = start_node;
    Json::Value goal_node(Json::arrayValue);
    for (unsigned int i=0; i < s_fin.size(); i++)
    {
        goal_node.append(s_fin[i]);
    }
    res_root["goal"] = goal_node;
    /*****
     * Setup OMPL
     *****/
    /* Make our configuration space and set the bounds on each module's angles 
     * and velocities.
     */
    int space_dim = 2*num_modules;
    ob::StateSpacePtr space(new ob::RealVectorStateSpace());
   /* Angular Position dimensions */
    for (int i=0; i < num_modules; i++)
    {
        ob::RealVectorStateSpace *rs = space->as<ob::RealVectorStateSpace>();
        rs->addDimension(ch.get_link(i).get_joint_min(),
                         ch.get_link(i).get_joint_max());
    }

    /* Angular Velocity dimensions */
    for (int i=0; i < num_modules; i++)
    {
        ob::RealVectorStateSpace *rs = space->as<ob::RealVectorStateSpace>();
        rs->addDimension(sets.max_joint_vel, sets.min_joint_vel);
    }

    /* Make our control space, which is one bound direction for each joint (Torques) */
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, num_modules));
    ob::RealVectorBounds cbounds(num_modules);
    for (int i=0; i < num_modules; i++)
    {
        cbounds.setLow(i, -ch.get_link(i).get_torque_max());
        cbounds.setHigh(i, ch.get_link(i).get_torque_max());
    }
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    /*
     * Use OMPL's built in setup mechanism instead of allocating
     * state space information and problem defintion pointers on my own
     */
    boost::shared_ptr<oc::SimpleSetup> ss_p(new oc::SimpleSetup(cspace));

    // TODO: Write an actual state validity checker!
    ss_p->setStateValidityChecker(boost::bind(&ckbot::CK_ompl::stateValidityChecker,
                                                 &(*rate_machine_p),
                                                 ss_p->getSpaceInformation(),
                                                 _1));

    /* Setup and get the dynamics of our system into the planner */
    oc::ODEBasicSolver<> odeSolver(ss_p->getSpaceInformation(),
                                   boost::bind(&ckbot::CK_ompl::CKBotODE,
                                               &(*rate_machine_p), _1, _2, _3));

    ss_p->setStatePropagator(odeSolver.getStatePropagator());

    /* Define the start and end configurations */
    ob::ScopedState<ob::RealVectorStateSpace> start(ss_p->getSpaceInformation());
    /* TODO: Custom goal class with custom distanceGoal(...) method */
    ob::ScopedState<ob::RealVectorStateSpace> goal(ss_p->getSpaceInformation());
    for (int i = 0; i < 2*num_modules; ++i)
    {
        start[i] = s0[i];
        goal[i] = s_fin[i];
    }
    start.print(std::cout);
    goal.print(std::cout);

    ss_p->setStartState(start);
    ss_p->setGoalState(goal, sets.threshhold);
    /* Initialize the correct planner (possibly specified on cmd line) */
    ob::PlannerPtr planner;
    planner = get_planner(ss_p->getSpaceInformation(), sets.planner);
    ss_p->setPlanner(planner);

    /* Allow this range of number of steps in our solution */
    ss_p->getSpaceInformation()->setMinMaxControlDuration(sets.min_control_steps, sets.max_control_steps);
    ss_p->getSpaceInformation()->setPropagationStepSize(sets.dt);

    /* Tell SimpleSetup that we've given it all of the info, and that
     * it should distribute parameters to the different components 
     * (mostly, fill in the planner parameters.) 
     */
    ss_p->setup();

    /* Debug printing section for information having to do with the setup
     * of the planner and any of its components.
     * This should be the only debug printing section for this purpose.
     */
    if (sets.debug)
    {
        /* Have the chain in our rate machine describe itself */
        out_file << rate_machine_p->get_chain().describe_self();

        /* Print the planner start and goal states */
        out_file << "Planner Start and Goal states: " << std::endl;
        const ob::ProblemDefinitionPtr pProbDef = planner->getProblemDefinition();
        pProbDef->print(out_file);

        /* Print the current planner parameter list */
        std::map< std::string, std::string > params;
        planner->params().getParams(params); 

        out_file << "Planner Parameter list: " << std::endl; 
        std::map< std::string, std::string >::iterator paramIt;
        for (paramIt = params.begin(); paramIt != params.end(); paramIt++)
        {
            out_file << "    " << paramIt->first << ", " << paramIt->second << std::endl;
        }

        /* Print the control space bounds */
        const oc::ControlSpacePtr pControl = ss_p->getControlSpace();
        const ob::RealVectorBounds bounds = pControl->as<oc::RealVectorControlSpace>()->getBounds();
        std::vector<double> low = bounds.low;
        std::vector<double> high = bounds.high;

        out_file << "Control Space bounds: " << std::endl; 
        for (unsigned int i=0; i<low.size(); i++)
        {
            out_file << "    Link " << i+1 
                << ": Low: " << low[i]
                << " High: " << high[i] << std::endl;
        }

        /* Print the configuration space bounds */
        const ob::RealVectorBounds cbounds = space->as<ob::RealVectorStateSpace>()->getBounds();
        out_file << "The Configuration bounds are: " << std::endl;
        std::vector<double> clow = cbounds.low;
        std::vector<double> chigh = cbounds.high;
        for (unsigned int i=0; i<clow.size(); i++)
        {
            out_file << "    Dof" << i+1
                << ": Low: " << clow[i]
                << " High: " << chigh[i] << std::endl;
        }

        /* Print information about how the planner will interface with the dynamics engine */
        out_file << "The progagationStepSize is: " 
            << ss_p->getSpaceInformation()->getPropagationStepSize()
            << std::endl << "    The Minimum number of steps is: " 
            << ss_p->getSpaceInformation()->getMinControlDuration() << std::endl 
            << "    The Max number of steps is: " 
            << ss_p->getSpaceInformation()->getMaxControlDuration() << std::endl;

        /* Print information about the ODE Solver */
        out_file << "The ODE Step size is: " << odeSolver.getIntegrationStepSize() << std::endl;
    }

    if (!run_planner(ss_p, sets, res_root))
    {
        std::cerr << "Error running simulation!" << std::endl;
        return boost::shared_ptr<oc::SimpleSetup>();
    }

    return ss_p;
}

bool
run_planner(boost::shared_ptr<oc::SimpleSetup> ss_p, struct sim_settings sets, Json::Value& res_root)
{

    bool solve_status = false;
    if (ss_p->solve(sets.max_sol_time))
    {
        solve_status = true;
        save_sol(ss_p, sets, res_root);
        if (sets.save_full_tree)
        {
            save_full_tree(ss_p, res_root); 
        }
    }
    return solve_status;
}

ob::PlannerPtr
get_planner(oc::SpaceInformationPtr si, enum planners plan)
{
    ob::PlannerPtr planner;
    if (plan == RRT)
    {
        ob::PlannerPtr p_temp(new oc::RRT(si));
        planner = p_temp;
    }
    else if (plan == KPIECE1)
    {
        ob::PlannerPtr p_temp(new oc::KPIECE1(si));
        planner = p_temp;
    }

    return planner;
}


/*
 * Output solution to file in json format
 */
bool
save_sol(boost::shared_ptr<oc::SimpleSetup> ss_p, struct sim_settings sets, Json::Value& res_root) 
{
    const ob::PlannerPtr planner = ss_p->getPlanner();
    const ob::ProblemDefinitionPtr prob_def = planner->getProblemDefinition();
    //const ob::RealVectorStateSpace::StateType *start = prob_def->getStartState(0u)->as<ob::RealVectorStateSpace::StateType>();
    const ob::GoalPtr& goal_ptr = prob_def->getGoal();
    const oc::PathControl& sol_path(ss_p->getSolutionPath());
    if (sets.debug)
    {
        sol_path.print(std::cout);
    }
    unsigned int num_modules = (*(ss_p->getStateSpace())).as<ob::RealVectorStateSpace>()->getDimension()/2;
    std::vector<double> time(sol_path.getStateCount());
    std::vector<double> dt(sol_path.getStateCount()-1);

    /* Note: We're assuming that we're already in a json dictionary */
    Json::Value controls(Json::arrayValue);

    time[0] = 0.0;
    for (unsigned int i=0; i < sol_path.getStateCount(); ++i)
    {
        if (i != 0)
        {
            Json::Value this_control(Json::objectValue);
            const ob::RealVectorStateSpace::StateType& s_minus = *sol_path.getState(i-1)->as<ob::RealVectorStateSpace::StateType>();
            const ob::RealVectorStateSpace::StateType& s = *sol_path.getState(i)->as<ob::RealVectorStateSpace::StateType>();

            dt[i-1] = sol_path.getControlDuration(i-1); // Control Duration to go from state i-1 to i;
            time[i] = time[i-1]+dt[i-1]; // Time at this step
            this_control["start_state_index"] = Json::Value(i-1);
            this_control["end_state_index"] = Json::Value(i);

            Json::Value start(Json::arrayValue);
            for (unsigned int j=0; j<2*num_modules; ++j)
            {
                start.append(Json::Value(s_minus[j]));
            }

            Json::Value end_state(Json::arrayValue);
            for (unsigned int j=0; j<2*num_modules; ++j)
            {
                end_state.append(Json::Value(s[j]));
            }

            this_control["start_state"] = start;
            this_control["end_state"] = end_state;
            this_control["start_time"] = Json::Value(time[i-1]);
            this_control["end_time"] = Json::Value(time[i]);
            this_control["dt"] = Json::Value(dt[i-1]);

            const double* c = sol_path.getControl(i-1)->as<oc::RealVectorControlSpace::ControlType>()->values;
            Json::Value this_control_vals(Json::arrayValue);
            for (unsigned int j=0; j<num_modules; ++j)
            {
                this_control_vals.append(Json::Value(c[j]));
            }
            this_control["control"] = this_control_vals;

            controls.append(this_control);
        }
    }

    res_root["controls"] = controls;
    return true;
}


/* Write everything needed to re-construct the planner exploration tree, with controls, 
 * to an output file.
 *
 * TODO: Re-write this function.  Make a nice function for outputting a state vector in
 *       json format, or something.  Half of this function in it's current form is
 *       repetition of the same code on a different structure.
 */
bool
save_full_tree(boost::shared_ptr<oc::SimpleSetup> ss_p, Json::Value& res_root)
{
    oc::RRT *kPlanner = ss_p->getPlanner()->as<oc::RRT>();
    oc::PlannerData data;
    kPlanner->getPlannerData(data);
    Json::Value tree(Json::objectValue);

    Json::Value states(Json::arrayValue);

    /* An array of the nodes (states) in the planng tree */
    unsigned int dimension = data.si->getStateDimension();
    std::vector<const ob::State*> state_vec = data.states;
    for (unsigned int i=0; i < state_vec.size(); i++)
    {
        Json::Value this_state(Json::arrayValue);
        const ob::RealVectorStateSpace::StateType *real_state = (state_vec[i])->as<ob::RealVectorStateSpace::StateType>();
        for (unsigned int j=0; j < dimension; j++)
        {
            this_state.append(Json::Value((*real_state)[j]));
        }
        states.append(this_state);
    }

    /* For each i, an array of the dictionaries of the format
     * {state: <num>, control: [control array]}
     * where <num> is the state to which i is connected and
     * [control array] is the control that brings the system from
     * state i to state <num>.
     * The 'i's here correspond to the same 'i's in the states array before this.
     */
    Json::Value conns(Json::arrayValue);
    unsigned int edge_indicies = data.edges.size();
    for (unsigned int i=0; i < edge_indicies; i++)
    {
        Json::Value i_to_j_edges(Json::arrayValue);
        unsigned int edge_count = (data.edges[i]).size();
        for (unsigned int j=0; j < edge_count; j++)
        {
            Json::Value this_conn(Json::objectValue);
            this_conn["state"] = Json::Value(data.edges[i][j]);
            Json::Value this_control(Json::arrayValue);
            const oc::RealVectorControlSpace::ControlType *control = (data.controls[i][j])->as<oc::RealVectorControlSpace::ControlType>();
            unsigned int control_dim = ss_p->getSpaceInformation()->getControlSpace()->getDimension();
            for (unsigned int k=0; k < control_dim; k++)
            {
                this_control.append(Json::Value((*control)[k]));
            }
            this_conn["control"] = this_control;
            i_to_j_edges.append(this_conn);
        }
        conns.append(i_to_j_edges);
    }

    tree["states"] = states;
    tree["connections"] = conns;
    res_root["tree"] = tree;
    return true;
}

bool
fill_start_and_goal(const Json::Value& sim_root,
                    std::vector<double>& s0,
                    std::vector<double>& s_fin)
{
    if((! sim_root["start"].isArray()) ||
       (! sim_root["goal"].isArray()))
    {
        std::cerr << "In simulation file the start and " <<
                     "goal positions must be Json arrays." << std::endl;
        return false;
    }

    if ((sim_root["start"].size() != s0.size()) ||
        (sim_root["goal"].size() != s_fin.size()))
    {
        std::cerr << "The start and goal positions in the sim file " <<
                     "must have the same dimension as the chain " <<
                     "suggests (ie: #modules*2)." << std::endl;
        return false;
    }

    for (unsigned int i = 0; i < s0.size(); ++i)
    {
       s0[i] = sim_root["start"][i].asDouble();
       s_fin[i] = sim_root["goal"][i].asDouble();
    }
    return true;
}

bool
parse_options(int ac, char* av[], boost::program_options::variables_map& vm, struct sim_settings& sets)
{
    /* Parse options and execute options */
    namespace po = boost::program_options;
    try
    {
        po::options_description desc("Usage:");
        desc.add_options()
            ("help", "Give non-sensical help.")
            ("dir", po::value<std::string>(&sets.sim_dir),
               "Set the directory in which the simulation to run exists")

            ("time",
              po::value<float>(&sets.max_sol_time),
              "Set the maximum runtime of the solver")

            ("min_control_steps",
              po::value<unsigned int>(&sets.min_control_steps),
              "Set the minimum number of steps OMPL applies controls for")

            ("max_control_steps",
              po::value<unsigned int>(&sets.max_control_steps),
              "Set the maximum number of steps OMPL applies controls for")

            ("dt",
              po::value<double>(&sets.dt),
              "Set the timestep resolution OMPL uses")

            ("max_torque",
              po::value<double>(&sets.max_torque),
              "Set the maximum torque a link can exert at its joint.")

            ("min_torque",
              po::value<double>(&sets.min_torque),
              "Set the minimum torque a link can exert at its joint.")

            ("threshhold",
             po::value<double>(&sets.threshhold),
             "Set the goal threshold")

            ("no_tree", "Don't the entire planning tree.")

            ("planner",
              po::value<unsigned int>(),
              "Set the planner: (0=RRT, 1=KPIECE)")

            ("collisions",
             "Use simple collision checking.  Requires world.txt in sim dir")

            ("debug",
              po::value<unsigned int>(&sets.debug),
             "Turn on debugging output.")
        ;

        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            return 1;
        }
        if (vm.count("planner"))
        {
            sets.planner = static_cast<enum planners>(vm["planner"].as<unsigned int>());
        }
        if (vm.count("no_tree"))
        {
            sets.save_full_tree = false;
        }
        if (vm.count("collisions"))
        {
            sets.collisions = true;
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "Exception of unknown type!" << std::endl;
        return false;
    }
    return true;
}

