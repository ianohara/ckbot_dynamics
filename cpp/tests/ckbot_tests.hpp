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

struct ckbot::module_description _TEST_MODULE_DESC = {
    1.0,
    Eigen::Vector3d(0,0,1),    /* forward_joint_axis */
    Eigen::Vector3d(-0.5,0,0), /* r_im1 */
    Eigen::Vector3d(0.5,0,0),  /* r_ip1 */
    Eigen::Matrix3d::Identity(), /* I_cm */
    Eigen::Matrix3d::Identity(), /* R_jts */
    ckbot::rotY(M_PI/2),         /* Init rotation */
    12.0,                       /* mass */
    -1.57,
    1.57,
    1.0
};

class ModuleTestSuite : public CxxTest::TestSuite
{
        ckbot::module_link *mp_;
        ckbot::module_link *def_;

    public:
        void setUp()
        {
            mp_ = new ckbot::module_link();
            def_ = new ckbot::module_link(_TEST_MODULE_DESC);
        }
        void tearDown()
        {
            delete mp_;
            delete def_;
        }

        void test_sets_gets()
        {
            TS_ASSERT_DELTA(mp_->get_q(), 0.0, EPS);
            mp_->set_q(M_PI);
            TS_ASSERT_DELTA(mp_->get_q(), M_PI, EPS);

            TS_ASSERT_DELTA(mp_->get_qd(), 0.0, EPS);
            mp_->set_qd(M_PI);
            TS_ASSERT_DELTA(mp_->get_qd(), M_PI, EPS);

            TS_ASSERT_DELTA(mp_->get_damping(), 0.0, EPS);
            mp_->set_damping(0.1);
            TS_ASSERT_DELTA(mp_->get_damping(), 0.1, EPS);

            TS_ASSERT_DELTA(mp_->get_joint_max(), 0.0, EPS);
            TS_ASSERT_DELTA(mp_->get_joint_min(), 0.0, EPS);
            TS_ASSERT_DELTA(mp_->get_torque_max(), 0.0, EPS);

            /* Test the module that was initialized with a descriptions struct */
            TS_ASSERT_DELTA(def_->get_mass(), 12.0, EPS);
            TS_ASSERT_DELTA(def_->get_joint_max(), 1.57, EPS);
            TS_ASSERT_DELTA(def_->get_joint_min(), -1.57, EPS);
            TS_ASSERT_DELTA(def_->get_torque_max(), 1.0, EPS);
            TS_ASSERT((def_->get_r_ip1()).isApprox(_TEST_MODULE_DESC.r_ip1));
            TS_ASSERT((def_->get_r_im1()).isApprox(_TEST_MODULE_DESC.r_im1));
            TS_ASSERT((def_->get_I_cm()).isApprox(_TEST_MODULE_DESC.I_cm));
            TS_ASSERT((def_->get_forward_joint_axis()).isApprox(_TEST_MODULE_DESC.forward_joint_axis));
            TS_ASSERT((def_->get_R_jts()).isApprox(_TEST_MODULE_DESC.R_jts));
            TS_ASSERT_DELTA(def_->get_damping(), _TEST_MODULE_DESC.damping, EPS);
        }
        void test_rots()
        {
            TS_ASSERT(ckbot::rotX(0.0).isApprox(Eigen::Matrix3d::Identity()));
            TS_ASSERT(ckbot::rotY(0.0).isApprox(Eigen::Matrix3d::Identity()));
            TS_ASSERT(ckbot::rotZ(0.0).isApprox(Eigen::Matrix3d::Identity()));

            Eigen::Matrix3d rotx_piover2;
            rotx_piover2 << 1.0,  0,    0,
                              0,  0,   -1.0,
                              0,  1.0,  0;

            Eigen::Matrix3d roty_piover2;
             roty_piover2 << 0,   0,    1.0,
                             0,   1.0,  0.0,
                            -1.0, 0.0,  0;

            Eigen::Matrix3d rotz_piover2;
            rotz_piover2 << 0,  -1.0, 0,
                            1.0, 0,   0,
                            0,   0,   1.0;

            TS_ASSERT(ckbot::rotX(M_PI/2).isApprox(rotx_piover2));
            TS_ASSERT(ckbot::rotY(M_PI/2).isApprox(roty_piover2));
            TS_ASSERT(ckbot::rotZ(M_PI/2).isApprox(rotz_piover2));
        }
        void test_cross_mat()
        {
            Eigen::Vector3d r1 = Eigen::Vector3d::Random();
            Eigen::Vector3d r2 = Eigen::Vector3d::Random();

            Eigen::Matrix3d cmat_r1 = ckbot::get_cross_mat(r1);
            Eigen::Matrix3d cmat_r2 = ckbot::get_cross_mat(r2);

            TS_ASSERT((r1.cross(r2)).isApprox(cmat_r1*r2));
            TS_ASSERT((r2.cross(r1)).isApprox(cmat_r2*r1));
        }
        void test_body_trans()
        {
            // TODO: TEST THIS GUY!
            Eigen::Vector3d world_vo(0,0,0);
            Eigen::Vector3d oLc(1,0,0);
            Eigen::Vector3d oWc(0,0,1);
        }
        
};
