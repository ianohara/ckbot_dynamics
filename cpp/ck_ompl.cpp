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

#include<iostream>
#include<cmath>

/*
 * This code implements the functions needed by differential (control) planners
 * in OMPL.
 *
 */
#include<ompl/control/ODESolver.h>
#include<ompl/control/spaces/RealVectorControlSpace.h>
#include<ompl/control/Control.h>
#include<ompl/base/spaces/RealVectorStateSpace.h>
#include<ompl/base/State.h>
#include<ompl/base/SpaceInformation.h>

namespace oc = ompl::control;
namespace ob = ompl::base;

#include"ckbot.hpp"
#include"ck_ompl.hpp"

/*
 * Take a json chain subtree, which is an array of module dictionaries,
 * and turn it into a usable chain object populated by the module defined
 * in the module dictionaries.
 */
boost::shared_ptr<ckbot::CK_ompl>
ckbot::setup_ompl_ckbot(Json::Value& chain_root, std::ostream& out_file)
{
    Json::Value chain_array = chain_root["chain"];
    int num_modules = chain_array.size();
    /* This will never be explicitly freed, just let program end do it. */
    ckbot::module_link* modules_for_chain = new ckbot::module_link[num_modules];
    for (unsigned int link=0; link<num_modules; ++link)
    {
        if (! ckbot::fill_module(chain_array[link], &modules_for_chain[link]))
        {
            return boost::shared_ptr<ckbot::CK_ompl>();
        }
    }
    /* Again, never explictly freed.  Let the program run till death! */
    ckbot::chain *ch = new ckbot::chain(modules_for_chain, num_modules);
    /* Chain rate store a reference to a chain, so this is right memory-wise
     * (right?) de-ref pointer, rate_machine looks for a reference so the
     * dereferenced chain isn't passed as a copy, but instead as a reference.
     * Think that's right...
     */
    boost::shared_ptr<ckbot::CK_ompl> rate_machine_p(new ckbot::CK_ompl(*ch));
    return rate_machine_p;
};

ckbot::CK_ompl::CK_ompl(ckbot::chain& ch) :
        chain_rate(ch),
        world(boost::shared_ptr<World>()) /* Null */
{
}

ckbot::CK_ompl::~CK_ompl()
{
}

bool
ckbot::CK_ompl::setWorld(boost::shared_ptr<World> w)
{
    world = w;
}

bool
ckbot::CK_ompl::stateValidityChecker(const ob::SpaceInformationPtr &si, const ob::State *s)
{
    if (! (si->satisfiesBounds(s))) {
       return false;
    }

    if (world)
    {
        const double MOD_SPHERE_RADIUS = 0.05; /* [m] */
        /* World exists, use collision checking */
        const double *sVals = s->as<ob::RealVectorStateSpace::StateType>()->values;
        int num_links = c.num_links();
        int slen = 2*num_links;
        std::vector<double> q(num_links);
        std::vector<double> qd(num_links);
        for (int i = 0; i < num_links; i++)
        {
            q[i] = sVals[i];
            qd[i] = sVals[num_links+i];
        }
        c.propogate_angles_and_rates(q,qd); /* TODO: Does this need to be called? */
        for (int i = 0; i < num_links; i++)
        {
            Sphere tmpS;
            tmpS.loc = c.get_link_r_cm(i);
            tmpS.r = MOD_SPHERE_RADIUS;
            bool colliding = world->isColliding(tmpS);
            if (colliding) {
                return false; /* Not a valid state */
            }
        }
    }
    /* No collisions or No collision world, in which case
     * all states are valid 
     */
    return true;
}

void
ckbot::CK_ompl::CKBotODE(const oc::ODESolver::StateType& s,
                         const oc::Control* con,
                         oc::ODESolver::StateType& sdot)
{
    const int N = c.num_links();
    const double *input = con->as<oc::RealVectorControlSpace::ControlType>()->values;
    std::vector<double> T(N);

    for (int i = 0; i < N; i++)
    {
        T[i] = input[i];
    }

    std::vector<double> sdot_vec(2*N);
    sdot_vec = calc_rate(static_cast<std::vector<double> >(s), T);

    for (int i = 0; i < 2*N; i++)
    {
        sdot[i] = sdot_vec[i];
    }
}

ckbot::EuclDistGoalState::EuclDistGoalState(const ob::SpaceInformationPtr &si,
                                            ckbot::chain &chain) :
    ob::GoalState(si), /* Parent Constructor */
    ch(chain),
    si(si)
{
}

/* Distance metric classes */
double
ckbot::EuclDistGoalState::distanceGoal(const ob::State *s) const
{
    const double *gVals = state_->as<ob::RealVectorStateSpace::StateType>()->values;
    const double *sVals = s->as<ob::RealVectorStateSpace::StateType>()->values;
    const int num_links = ch.num_links();
    double sumSqrs = 0;
    for (int i = 0; i < 2*num_links; i++)
    {
        double diff = gVals[i] - sVals[i];
        sumSqrs += diff*diff;
    }
    double dist = sqrt(sumSqrs);
    return dist;
}

ckbot::EndLocGoalState::EndLocGoalState(const ob::SpaceInformationPtr &si,
                                        ckbot::chain &chain,
                                        double x,
                                        double y,
                                        double z) :
            ob::GoalState(si), /* Parent contstructor */
            ch(chain),
            si(si),
            x_(x),
            y_(y),
            z_(z)
{
}

double
ckbot::EndLocGoalState::distanceGoal(const ob::State *s) const
{
    static double minDist = 100000000.0;
    const double *sVals = s->as<ob::RealVectorStateSpace::StateType>()->values;
    const int num_links = ch.num_links();
    int slen = 2*num_links;
    std::vector<double> q(num_links);
    std::vector<double> qd(num_links);
    for (int i = 0; i < num_links; i++)
    {
        q[i] = sVals[i];
        qd[i] = sVals[num_links+i];
    }
    ch.propogate_angles_and_rates(q,qd);

    Eigen::Vector3d r_end = ch.get_link_r_tip(num_links-1);
    double dist = sqrt((r_end[0] - x_)*(r_end[0]-x_)
                      +(r_end[1] - y_)*(r_end[1]-y_)
                      +(r_end[2] - z_)*(r_end[2]-z_));
    if (dist < minDist) {
        minDist = dist;
        std::cout << "Found a new min end effector distance from goal: " << minDist << std::endl;
        //std::cout << "r_end = " << r_end.transpose() << " (dist_origin = " << r_end.norm() << ")" << std::endl;
        //std::cout << "Goal = [" << x_ << ", " << y_ << ", " << z_ << "]" << std::endl;
    }

    return dist;
}

/**** KPIECE Projection classes ****/

/* EndLocAndAngVelProj uses a 4 dimensional projection where the first
 * three dimensions are the x,y,z positions of the end of the chain
 * and the fourth dimension is the sum of the squares of the angular
 * velocities of the joints
 */
ckbot::EndLocAndAngVelProj::EndLocAndAngVelProj(const ob::SpaceInformationPtr &si,
        const ob::StateSpacePtr &space,
        ckbot::chain &chain) :
    ob::ProjectionEvaluator(space),
    si(si),
    sp(space),
    ch(chain)
{
}

unsigned int
ckbot::EndLocAndAngVelProj::getDimension(void) const
{
    return 4u;
}

void
ckbot::EndLocAndAngVelProj::project(const ob::State *state,
        ob::EuclideanProjection &proj) const
{
    const double *sVals = state->as<ob::RealVectorStateSpace::StateType>()->values;
    const int num_links = ch.num_links();
    int slen = 2*num_links;
    std::vector<double> q(num_links);
    std::vector<double> qd(num_links);
    for (int i = 0; i < num_links; i++)
    {
        q[i] = sVals[i];
        qd[i] = sVals[num_links+i];
    }
    ch.propogate_angles_and_rates(q,qd);

    double sumSqrVels = 0;
    for (int i = 0; i < qd.size(); i++)
    {
        sumSqrVels += qd[i]*qd[i];

    }
    Eigen::Vector3d r_end = ch.get_link_r_tip(num_links-1);
    //DEBUG std::cout << "End is at: " << r_end.transpose() << std::endl;
    proj[0] = r_end[0];
    proj[1] = r_end[1];
    proj[2] = r_end[2];
    proj[3] = sqrt(sumSqrVels);
}

/* EndDistSqrProj uses a 1 dimensional projection where the dimension is the 
 * square of the distance of the end effector to the origin.
 */
ckbot::EndDistSqrProj::EndDistSqrProj(const ob::SpaceInformationPtr &si,
        const ob::StateSpacePtr &space,
        ckbot::chain &chain) :
    ob::ProjectionEvaluator(space),
    si(si),
    sp(space),
    ch(chain)
{
}

unsigned int
ckbot::EndDistSqrProj::getDimension(void) const
{
    return 1u;
}

void
ckbot::EndDistSqrProj::project(const ob::State *state,
        ob::EuclideanProjection &proj) const
{
    const double *sVals = state->as<ob::RealVectorStateSpace::StateType>()->values;
    const int num_links = ch.num_links();
    int slen = 2*num_links;
    std::vector<double> q(num_links);
    std::vector<double> qd(num_links);
    for (int i = 0; i < num_links; i++)
    {
        q[i] = sVals[i];
        qd[i] = sVals[num_links+i];
    }
    ch.propogate_angles_and_rates(q,qd);

    Eigen::Vector3d r_end = ch.get_link_r_tip(num_links-1);
    //DEBUG std::cout << "End is at: " << r_end.transpose() << std::endl;
    double dSq = r_end[0]*r_end[0] + r_end[1]*r_end[1] + r_end[2]*r_end[2];
    proj[0] = dSq;
}
