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


#ifndef _CKBOT_HPP
#define _CKBOT_HPP

#include<vector>
#include<iostream>
#include<fstream>
#define _USE_MATH_DEFINES
#include<math.h>

#include<Eigen/Core>
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include <json/json.h>

namespace ckbot
{
    struct module_description {
        double damping;
        double rotor_cogging;
        double stator_cogging;
        double cogging_offset;
        Eigen::Vector3d forward_joint_axis;
        Eigen::Vector3d r_im1;
        Eigen::Vector3d r_ip1;
        Eigen::Matrix3d I_cm;
        Eigen::Matrix3d R_jts;
        Eigen::Matrix3d init_rotation;
        double m;
        double joint_min;
        double joint_max;
        double torque_max;
    };

    template <typename Derived>
    Json::Value
    eigen_to_json(const Eigen::EigenBase<Derived>& mat)
    {
        Json::Value outer(Json::arrayValue);
        for (int m=0; m < mat.rows(); ++m)
        {
            Json::Value inner(Json::arrayValue);
            for (int n=0; n < mat.cols(); ++n)
            {
                inner.append((mat.derived())(m,n));
            }
            outer.append(inner);
        }
        return outer;
    }

    Eigen::Matrix3d rotX(double phi);
    Eigen::Matrix3d rotY(double phi);
    Eigen::Matrix3d rotZ(double phi);
    Eigen::Matrix3d get_cross_mat(Eigen::Vector3d r);
    Eigen::MatrixXd get_body_trans(Eigen::Vector3d r);

    class module_link
    {
        public:
            module_link(void);
            module_link(struct module_description m);
            module_link(const module_link& source);
            ~module_link(void);
 
            module_link& operator=(module_link& source);

            Eigen::RowVectorXd get_joint_matrix(void) const;

            Json::Value describe_self(void);

            double get_q(void) const;
            double get_qd(void) const;
            double get_damping(void) const;
            double get_rotor_cogging(void) const;
            double get_stator_cogging(void) const;
            double get_cogging_offset(void) const;
            double get_mass(void) const;
            double get_joint_max(void) const;
            double get_joint_min(void) const;
            double get_torque_max(void) const;

            void set_q(double q);
            void set_qd(double qd);
            void set_damping(double damping);

            Eigen::Vector3d get_forward_joint_axis(void) const;
            Eigen::Vector3d get_r_im1(void) const;
            Eigen::Vector3d get_r_ip1(void) const;

            Eigen::Matrix3d get_I_cm(void) const;
            Eigen::Matrix3d get_R_jts(void) const;

            Eigen::Matrix3d get_init_rotation(void) const;

        protected:
            double q_;
            double qd_;
            double damping_;
            double rotor_cogging_;
            double stator_cogging_;
            double cogging_offset_;
            Eigen::Vector3d forward_joint_axis_;
            Eigen::Vector3d r_im1_;
            Eigen::Vector3d r_ip1_;
            Eigen::Matrix3d I_cm_;
            Eigen::Matrix3d R_jts_;
            Eigen::Matrix3d init_rotation_;
            double m_;
            double joint_min_;
            double joint_max_;
            double torque_max_;
    };

    bool fill_module(const Json::Value&, ckbot::module_link*);

    class chain
    {
    protected:
        const int N_;

    public:
        chain(module_link chain_modules[], int num_modules);
        chain(const chain& other);
        ~chain(void);

        module_link& get_link(int i);
        Eigen::Matrix3d get_current_R(int i);
        int num_links(void) const;
        void propogate_angles_and_rates(std::vector<double> q, std::vector<double> qd);

        Eigen::Vector3d get_angular_velocity(int i);
        Eigen::Vector3d get_linear_velocity(int i);
        Eigen::Vector3d get_link_r_cm(int i);
        Eigen::Vector3d get_link_r_tip(int i);
        Eigen::Vector3d get_link_r_base(int i);

        Json::Value describe_self(void);
        std::vector<module_link> links_;
    };

    class chain_rate
    {
    protected:
        chain &c;

        std::vector<double> G_all;
        std::vector<double> mu_all;
        std::vector<double> a_all;
        std::vector<double> sd;

   public:
        explicit chain_rate(chain& ch);
        ~chain_rate(void);

        std::vector<double> calc_rate(std::vector<double> s, std::vector<double> T);
        void tip_base_step(std::vector<double> s, std::vector<double> T);
        std::vector<double> base_tip_step(std::vector<double> s, std::vector<double> T);
        chain& get_chain(void);
    };
};

#endif /* _CKBOT_HPP */
