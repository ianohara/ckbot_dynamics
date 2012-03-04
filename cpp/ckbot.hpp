#ifndef _CKBOT_HPP
#define _CKBOT_HPP

#include<vectors>
#include<eigen3/Eigen/Dense>
#include<iostream>
#define _USE_MATH_DEFINES
#include<math.h>

namespace ckbot
{
    struct module_description {
        double damping;
        Eigen::Vector3d forward_joint_axis;
        Eigen::Vector3d r_im1;
        Eigen::Vector3d r_ip1;
        Eigen::Matrix3d I_cm;
        Eigen::Matrix3d R_jts;
        Eigen::Matrix3d init_rotation;
        double m;
    };

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

            Eigen::MatrixXd get_spatial_inertia_mat(void);
            Eigen::RowVectorXd get_joint_matrix(void) const;

            double get_q(void) const;
            double get_qd(void) const;
            double get_damping(void) const;
            double get_mass(void) const;
            void set_q(double q);
            void set_qd(double qd);

            Eigen::Vector3d get_forward_joint_axis(void) const;
            Eigen::Vector3d get_r_im1(void) const;
            Eigen::Vector3d get_r_ip1(vod) const;

            Eigen::Matrix3d get_I_cm(void) const;
            Eigen::Matrix3d get_R_jts(void) const;

            Eigen::Matrix3d get_init_rotation(void) const;
        protected:
            double q_;
            double qd_;
            double damping_;
            Eigen::Vector3d forward_joint_axis_;
            Eigen::Vector3d r_im1_;
            Eigen::Vector3d r_ip1_;
            Eigen::Matrix3d I_cm_;
            Eigen::Matrix3d R_jts_;
            Eigen::Matrix3d init_rotation_;
            double m_;
    };
    class chain
    {
    protected:
        const int N_;
        std::vector<module_link> links_;

    public:
        chain(module_link chain_modules[], int num_modules);
        ~chain(void);

        module_link& get_link(int i);
        Eigen::Matrix3d get_current_R(int i);
        int num_links(void);
        void propogate_angles_and_rates(std::vector<double> q, std::vector<double> qd);
        Eigen::Vector3d get_angular_velocity(int i);
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
        chain_rate(chain& ch);
        std::vector<double> calc_rate(std::vector<double> s, std::vector<double> T);
        void tip_base_step(std::vector<double> s, std::vector<double> T);
        std::vector<double> base_tip_step(std::vector<double> s, std::vector<double> T)
    };
}

#endif /* _CKBOT_HPP */
