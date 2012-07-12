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

#include <cxxtest/TestSuite.h>

#define _USE_MATH_DEFINES
#include<math.h>
#define EPS 0.0001

#include "../ckbot.hpp"
#include "../ck_ompl.hpp"
#include "../sim_util.hpp"
#include "../util.hpp"

#include<iostream>

static const int NUM_MODULES = 10;
static const double INITIAL_Q = 0.1;

extern struct ckbot::module_description _TEST_MODULE_DESC;

class ChainTestSuite : public CxxTest::TestSuite
{
        ckbot::module_link *mp_;
        ckbot::module_link *def_;
        ckbot::chain *ch_;

    public:
        void setUp()
        {
            mp_ = new ckbot::module_link();
            def_ = new ckbot::module_link(_TEST_MODULE_DESC);
            ckbot::module_link chain_modules[NUM_MODULES];
            for (int i=0; i < NUM_MODULES; i++)
            {
                chain_modules[i] = *def_;
            }
            ch_ = new ckbot::chain(chain_modules, NUM_MODULES);
            for (int i=0; i < NUM_MODULES; i++)
            {
                ch_->get_link(i).set_q(INITIAL_Q);
            }
        }
        void tearDown()
        {
            delete mp_;
            delete def_;
            delete ch_;
        }

        void test_initial_settings()
        {
            TS_ASSERT_EQUALS(ch_->num_links(), NUM_MODULES);
            for (int i=0; i < ch_->num_links(); i++)
            {
                TS_ASSERT_DELTA(ch_->get_link(i).get_q(), INITIAL_Q, EPS);
            }

            for (int i=0; i < ch_->num_links(); i++)
            {
                ch_->get_link(i).set_q(0.0);
                TS_ASSERT_DELTA(ch_->get_link(i).get_q(), 0.0, EPS);
                TS_ASSERT((ch_->get_link(i).get_R_jts()).isApprox(Eigen::Matrix3d::Identity()));
            }
            /* Angles are now 0, so all of the modules should be
             * at the "initial_rotaiton" rotation (as long as
             * R_jts is identity for all of the modules)
             */
            for (int i=0; i < ch_->num_links(); i++)
            {
                TS_ASSERT(ch_->get_current_R(i).isApprox(ch_->get_link(0).get_init_rotation()));
            }
        } 
        void test_geom_and_vels()
        {
            /* This test only makes sense with 0 joint angles and
             * R_jts = identity()
             */
            for (int i=0; i < ch_->num_links(); i++)
            {
                ch_->get_link(i).set_q(0.0);
                ch_->get_link(i).set_qd(0.0);
            }

            Eigen::Vector3d r_accum(0,0,0);
            Eigen::Matrix3d R_all = ch_->get_current_R(0); /* True if next is true */
            /* All joint angles must be 0 leading into this */
            for (int i=0; i < ch_->num_links(); i++)
            {
                TS_ASSERT_DELTA(ch_->get_link(i).get_q(), 0.0, EPS);
                TS_ASSERT(ch_->get_link(i).get_R_jts().isApprox(Eigen::Matrix3d::Identity()));

                /* r_accum at base */
                TS_ASSERT((ch_->get_link_r_base(i)).isApprox(R_all*r_accum));

                r_accum += -ch_->get_link(i).get_r_im1(); /* r_accum at CM */
                TS_ASSERT(ch_->get_link_r_cm(i).isApprox(R_all*r_accum));

                r_accum += ch_->get_link(i).get_r_ip1(); /* r_accum at tip */
                TS_ASSERT((ch_->get_link_r_tip(i)).isApprox(R_all*r_accum));
            }

            /* With all qd == 0, both omega and linear vel of each module
             * should be 0.
             */
            for (int i=0; i < ch_->num_links(); i++)
            {
                TS_ASSERT(ch_->get_angular_velocity(i).isApprox(Eigen::Vector3d::Zero()));
                TS_ASSERT(ch_->get_linear_velocity(i).isApprox(Eigen::Vector3d::Zero()));
            }

            /* Now with just the base qd set to 1, all modules
             * should have the same omega and linear should just
             * be omega.cross(r_cm)
             */
            ch_->get_link(0).set_qd(1.0);
            for (int i=0; i < ch_->num_links(); i++)
            {
                Eigen::Vector3d v_lin = ch_->get_linear_velocity(i);
                Eigen::Vector3d v_lin_manual = ch_->get_angular_velocity(i).cross(ch_->get_link_r_cm(i));
                TS_ASSERT(ch_->get_angular_velocity(i).isApprox(ch_->get_angular_velocity(0)));
                //std::cout << "Link " << i << std::endl;
                //std::cout << "\n  v_lin="; util::flat_print(v_lin); 
                //std::cout << "\n  v_lin_manual="; util::flat_print(v_lin_manual); std::cout << std::endl;
                TS_ASSERT(v_lin.isApprox(v_lin_manual));
            }
        }
};
