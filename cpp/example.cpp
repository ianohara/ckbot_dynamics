#include <eigen3/Eigen/Dense>
#include<vector>
#include<iostream>
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

    struct ckbot::module_description HT1 = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, init_rotation, mass};

    ckbot::module_link test_1 = ckbot::module_link(HT1);

    test_1.describe_self();

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

bool 
ckbotStateValidityCheckerFunction(const ompl::base::State *)
{
    return true;
}

void 
sim_test_2(void)
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

    struct ckbot::module_description HT1 = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, init_rotation, mass};

    ckbot::module_link test_2 = ckbot::module_link(HT1);

    ckbot::module_link chain_modules[] = {test_2, test_2};
    int num_modules = 2;

    ckbot::chain ch = ckbot::chain(chain_modules, num_modules);

    ckbot::CK_ompl rate_machine(ch);

    std::vector<double> s0(2*num_modules);
    std::fill(s0.begin(), s0.end(), 0.0);

    std::vector<double> T(num_modules);
    std::fill(T.begin(), T.end(), 0.0);

    /* Set up OMPL */
    /* Make our configuration space and set the bounds on each module's angles */
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2*num_modules));
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(-M_PI/2.0, M_PI/2.0);


    /* Make our control space, which is one bound direction for each joint (Torques) */
    ompl::control::ControlSpacePtr cspace(new ompl::control::RealVectorControlSpace(space, num_modules));

    ompl::base::RealVectorBounds cbounds(num_modules);
    cbounds.setLow(-10.0);
    cbounds.setHigh(10.0);

    cspace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds); // TODO (IMO): Arbitrary for now

    /* 
     * Use OMPL's built in setup mechanism instead of allocating state space information
     * and problem defintion pointers on my own
     */
    ompl::control::SimpleSetup ss(cspace);

    // TODO: Write an actual state validity checker!
    ss.setStateValidityChecker(boost::bind(&ckbotStateValidityCheckerFunction, _1));
   /* 
    ompl::base::PlannerPtr planner(new ompl::control::KPIECE1(ss.getSpaceInformation()));
    ss.setPlanner(planner);
*/
    /* Setup and get the dynamics of our system into the planner */
    // ompl::control::ODEBasicSolver<> odeSolver (ss.getSpaceInformation(), boost::bind<void>(&ckbot::CKBotODEFunc, _1, _2, _3, rate_machine));
    ompl::control::ODEBasicSolver<> odeSolver (ss.getSpaceInformation(), boost::bind<void>(&ckbot::CK_ompl::CKBotODE, rate_machine, _1, _2, _3));
    ss.setStatePropagator(odeSolver.getStatePropagator());

    /* Define the start and end configurations */
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(ss.getSpaceInformation());
    start[0]=start[1]=start[2]=start[3] = 0.0;

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(ss.getSpaceInformation());
    goal[0] = goal[2] = M_PI/2; // Angles
    goal[1] = goal[3] = 0; // Velocities

    ss.setStartAndGoalStates(start, goal);

    /* Allow more than 10 steps in our solution */
    ss.getSpaceInformation()->setMinMaxControlDuration(1, 100);

    ss.setup();
    std::cout << "The starting point is: " << start << "\n";
    std::cout << "The goal point is: ";
    ss.getGoal()->print();

    if(ss.solve(20.0))
    {
            ss.getSolutionPath().print(std::cout);
        /*
        ompl::control::PathControl& path(planner->getSolutionPath());
        for (unsigned int i=0; i<path.getStateCount(); ++i)
        {
            const ompl::base::RealVectorStateSpace::StateType& s0 = *path.getState(i)->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>;
            const ompl::base::RealVectorStateSpace::StateType& s1 = *path.getState(i)->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1);

            std::cout << "Controls: " << s1[0] << " " << s1[1] << " \n";
        }
        */
    }
}
