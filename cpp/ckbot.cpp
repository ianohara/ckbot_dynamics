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


#ifndef _CKBOT_CPP
#define _CKBOT_CPP

#include<vector>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include<fstream>
#define _USE_MATH_DEFINES
#include<math.h>
#include"ckbot.hpp"

Eigen::Matrix3d 
ckbot::rotY(double phi)
{
    Eigen::Matrix3d Ry;
    Ry << cos(phi), 0.0, sin(phi),
          0.0,      1.0, 0.0,
         -sin(phi), 0.0, 0.0;
    return Ry;
}
Eigen::Matrix3d 
ckbot::rotZ(double phi)
{
    /*std::cout << "Getting rotZ...\n";*/
    Eigen::Matrix3d Rz;
    Rz << cos(phi), -sin(phi), 0,
          sin(phi),  cos(phi), 0,
          0.0,       0.0,      1.0;
    return Rz;
}

Eigen::Matrix3d 
ckbot::get_cross_mat(Eigen::Vector3d r)
{
    /*std::cout << "Getting cross mat using " << r(1) << "...\n";*/
    Eigen::Matrix3d r_cross;
    r_cross << 0,   -r[2], r[1],
               r[2], 0,   -r[0],
              -r[1], r[0], 0;
    return r_cross;
}

Eigen::MatrixXd 
ckbot::get_body_trans(Eigen::Vector3d r)
{
    /*std::cout << "Getting body trans...\n";*/
    Eigen::MatrixXd phi(6,6);

    Eigen::Matrix3d r_cross;
    r_cross = get_cross_mat(r);

    phi << Eigen::Matrix3d::Identity(), r_cross,
           Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

    return phi;
}

/* The void constructor is needed in order to 
 * make STL vectors of module links.  In practice
 * this shouldn't be used because the defaults 
 * are non-sense.
 */
ckbot::module_link::module_link(void):
    q_(0.0),
    qd_(0.0),
    damping_(0.0),
    forward_joint_axis_(Eigen::Vector3d::Zero()),
    r_im1_(Eigen::Vector3d::Zero()),
    r_ip1_(Eigen::Vector3d::Zero()),
    I_cm_(Eigen::Matrix3d::Identity()),
    R_jts_(Eigen::Matrix3d::Identity()), 
    init_rotation_(Eigen::Matrix3d::Identity()),
    m_(0)
{
}

ckbot::module_link::module_link(struct module_description m): 
    q_(0.0),
    qd_(0.0),
    damping_(m.damping),
    forward_joint_axis_(m.forward_joint_axis),
    r_im1_(m.r_im1),
    r_ip1_(m.r_ip1),
    I_cm_(m.I_cm),
    R_jts_(m.R_jts),
    init_rotation_(m.init_rotation),
    m_(m.m),
    joint_min_(m.joint_min),
    joint_max_(m.joint_max),
    torque_max_(m.torque_max)
{
}

ckbot::module_link::module_link(const module_link& source):
    q_(source.get_q()),
    qd_(source.get_qd()),
    damping_(source.get_damping()),
    forward_joint_axis_(source.get_forward_joint_axis()),
    r_im1_(source.get_r_im1()),
    r_ip1_(source.get_r_ip1()),
    I_cm_(source.get_I_cm()),
    R_jts_(source.get_R_jts()),
    init_rotation_(source.get_init_rotation()),
    m_(source.get_mass()),
    joint_min_(source.get_joint_min()),
    joint_max_(source.get_joint_max()),
    torque_max_(source.get_torque_max())
{
}

ckbot::module_link::module_link& 
ckbot::module_link::operator=(module_link& source)
{
    damping_ = source.get_damping();
    forward_joint_axis_ = source.get_forward_joint_axis();
    r_im1_ = source.get_r_im1();
    r_ip1_ = source.get_r_ip1();
    I_cm_ = source.get_I_cm();
    R_jts_ = source.get_R_jts();
    init_rotation_ = source.get_init_rotation();
    m_ = source.get_mass();
    joint_max_ = source.get_joint_max();
    joint_min_ = source.get_joint_min();
    torque_max_ = source.get_torque_max();
}

/* Have a module describe itself, in valid json form, to a stream */
void
ckbot::module_link::describe_self(std::ostream& out)
{
    out << "\"module\": {" << std::endl << 
        "\"mass\": " << m_ << "," << std::endl <<
        "\"damping\": " << damping_ << "," << std::endl <<
        "\"joint_max\": " << joint_max_ << "," << std::endl <<
        "\"joint_min\": " << joint_min_ << "," << std::endl << 
        "\"torque_max\": " << torque_max_ << "," << std::endl <<
        "\"f_jt_axis\":" << std::endl;
    eigen_json_print(out, forward_joint_axis_);
    out << "," << std::endl;

    out << "\"r_im1\":" << std::endl;
    eigen_json_print(out, r_im1_); 
    out << "," << std::endl;

    out << "\"I_cm\":" << std::endl;
    eigen_json_print(out, I_cm_);
    out << "," << std::endl;

    out << "\"R_jts\":" << std::endl;
    eigen_json_print(out, R_jts_);
    out << "," << std::endl;

    out << "\"init_rotation\":" << std::endl;
    eigen_json_print(out, init_rotation_);
    
    out << "}" << std::endl;
}

ckbot::module_link::~module_link(void)
{
}

double
ckbot::module_link::get_joint_min(void) const
{
    return joint_min_;
}
double
ckbot::module_link::get_joint_max(void) const
{
    return joint_max_;
}
double
ckbot::module_link::get_torque_max(void) const
{
    return torque_max_;
}
    

/* Returns a module's 6x6 spatial
 * inertia matrix with respect to
 * its base-side joint.
 */
Eigen::MatrixXd 
ckbot::module_link::get_spatial_inertia_mat(void)
{
    Eigen::Matrix3d Jo;
    Eigen::MatrixXd M_spat(6,6);

    Eigen::Matrix3d L_tilde;
    L_tilde = get_cross_mat(-r_im1_);

    Jo = I_cm_ - m_*L_tilde*L_tilde;
    M_spat << Jo, m_*L_tilde,
             -m_*L_tilde, m_*(Eigen::Matrix3d::Identity());

    return M_spat;
}

/* Returns a 1x6 vector that maps changes in a modules
 * single DOF to spatial changes.  In the simple case here,
 * changes in a module's DOF are mapped to angular changes
 * in the body z-axis of the previous module (ie: rotation around
 * the base joint axis)
 */
Eigen::RowVectorXd 
ckbot::module_link::get_joint_matrix(void) const 
{
    Eigen::Vector3d Hprev(0,0,1);
    Eigen::Vector3d Ht;
    Ht = R_jts_.transpose()*Hprev;

    Eigen::RowVectorXd H(6);
    
    H << Ht(0), Ht(1), Ht(2), 0, 0, 0;
    return H;
}

/* Relative of a module with respect to its base side neighbor
 * in Radians.
 * [rad]
 */
double 
ckbot::module_link::get_q(void) const
{
    return q_;
}

/* Relative angular rate of a module with respect to its
 * base side neighbor. In radians/second.
 * [rad/s]
 */
double 
ckbot::module_link::get_qd(void) const
{
    return qd_;
}

/* Returns a 3x1 vector that defines
 * the joint connecting a module to its
 * tip side neighbor.  This needs to be a unit vector,
 * and should be written in the module's frame.
 */
Eigen::Vector3d
ckbot::module_link::get_forward_joint_axis(void) const
{
    return forward_joint_axis_;
}

/* Module frame 3x1 vector from a module's
 * CM to its base side joint. 
 * [m]
 */
Eigen::Vector3d 
ckbot::module_link::get_r_im1(void) const
{
    return r_im1_;
}

/* Module frame 3x1 vector from a module's
 * CM to its tip side joint
 * [m]
 */
Eigen::Vector3d 
ckbot::module_link::get_r_ip1(void) const
{
    return r_ip1_;
}

/* Returns a module's 3x3 inertia matrix written
 * in the module frame with respect to the
 * module's CM
 * [m^4]
 */
Eigen::Matrix3d
ckbot::module_link::get_I_cm(void) const
{
    return I_cm_;
}

/* TODO: Comment
 */
Eigen::Matrix3d 
ckbot::module_link::get_R_jts(void) const
{
    return R_jts_;
}

/* Returns the 3x3 Rotation matrix that defines
 * the initial world to module frame rotation of a module.
 *
 * This is ignored for modules that aren't
 * first in a chain.  It allows arbitrary orientaions
 * of the first link in a chain.
 */
Eigen::Matrix3d
ckbot::module_link::get_init_rotation(void) const
{
    return init_rotation_;
}

/* Returns the damping coefficient of a module's
 * base side (where its motor/DOF is) joint.  
 * [N*m*s]
 */
double
ckbot::module_link::get_damping(void) const
{
    return damping_;
}

/* Returns a module's mass.
 * [kg]
 */
double
ckbot::module_link::get_mass(void) const
{
    return m_;
}

/* Set a module's joint DOF angle.  This
 * is the angle between its base side neighbor and
 * itself.
 * [rad]
 */
void
ckbot::module_link::set_q(double q)
{
    q_ = q;
}

/* Set a module's joint DOF angular rate.  This
 * is the angular rate with respect to its
 * base side neighbor.
 */
void
ckbot::module_link::set_qd(double qd)
{
    qd_ = qd;
}


/* Form a new chain of modules.  The array of module_links, chain_modules,
 * needs to a length of num_modules.
 */
ckbot::chain::chain(module_link chain_modules[], int num_modules) : N_(num_modules), links_(num_modules)
{
    for (int link=0; link<num_modules; ++link)
    {
        links_[link] = chain_modules[link];
    }
}

ckbot::chain::~chain(void)
{
}

/*
 * Tell each module on the chain to describe itself (in chain order from base to tip)
 */
void 
ckbot::chain::describe_self(std::ostream& out)
{
    out << "---BEGIN CHAIN DESCRIPTION---" << std::endl;
    for (int i=0; i<N_; ++i)
    {
        out << "--Module " << i << "--" << std::endl;
        links_[i].describe_self(out);
    }
    out << "---END CHAIN DESCRIPTION---" << std::endl;
}


/* When walking up and down a chain, it is nice
 * to be able to define a sort of "current module"
 * reference. This serves the purpose. 
 */
ckbot::module_link& 
ckbot::chain::get_link(int i)
{
    return links_[i];
}

/* Returns a 3x3 rotation matrix.
 * Starting at the base of the chain (links_[0]) this
 * walks and composes module's rotations together to get
 * the rotation matrix that brings the i-th module from
 * its module frame to the world frame.
 */
Eigen::Matrix3d 
ckbot::chain::get_current_R(int i)
{
    Eigen::Matrix3d R;
    R = links_[0].get_init_rotation();

    for (int j = 0; j <= i; ++j)
    {
        R = R*rotZ(links_[j].get_q())*links_[j].get_R_jts();
    }
    return R;
}

int 
ckbot::chain::num_links(void)
{
    return N_;
}

/* 
 * If the propogate function needs to be thread safe, this method of
 * updating a chain's state (q,qd) needs to be changed.  Any
 * method that uses links_[] to get these two must then be changed to accept
 * references to vectors of q and qd.
 */
void 
ckbot::chain::propogate_angles_and_rates(std::vector<double> q, std::vector<double> qd)
{
    for (int i=0; i<N_; ++i)
    {
        links_[i].set_q(q[i]);
        links_[i].set_qd(qd[i]);
    }
}

/* Walks along the chain from base to the i-th module
 * adding each module's simple angular velocity (transformed to
 * the world frame) and then returning the compound angular velocity
 * of the i-th module that results.
 */
Eigen::Vector3d 
ckbot::chain::get_angular_velocity(int i)
{
    Eigen::Vector3d omega(0,0,0);
    Eigen::Matrix3d R;
    R = links_[0].get_init_rotation();

    Eigen::Vector3d tmp3vec(0,0,0);
    for (int cur=0; cur <= i; ++cur)
    {
        tmp3vec << 0,0, links_[cur].get_qd();
        omega += R*tmp3vec;
        R = R*rotZ(links_[cur].get_q())*links_[cur].get_R_jts();
    }
}

ckbot::chain_rate::chain_rate(chain& ch) : 
    c(ch), 
    G_all(6*ch.num_links()),
    mu_all(ch.num_links()),
    a_all(6*ch.num_links()),
    sd(2*ch.num_links())
{
}

ckbot::chain& ckbot::chain_rate::get_chain(void)
{
    return c;
}

/* Given a state vector of length 2*N corresponding to N joint angles
 * and N joint speeds and torque vector of length N, 
 * this fills in the 2*N length vector sd with
 * the corresponding joint speeds and joint accelerations.
 */
std::vector<double> 
ckbot::chain_rate::calc_rate(std::vector<double> s, std::vector<double> T)
{
    int N = c.num_links();

    std::vector<double> qdd(N);
    std::vector<double> q(N);
    std::vector<double> qd(N);

    // std::cout << "Separating out the state into its parts: \n";
    for(int i=0; i<N; ++i)
    {
       // std::cout<< "Link " << i << ": q=" << s[i] << " qd=" << s[N+i] << "\n";
        q[i] = s[i];
        qd[i] = s[N+i];
    }

    /* Update the chain's state to match the requested initial
     * conditions (s)
     */
    c.propogate_angles_and_rates(q,qd);

    tip_base_step(s, T);
    qdd = base_tip_step(s, T);
    
    for (int i=0; i < N; ++i)
    {
        /* Rate of change of positions are already in our state */
        sd[i] = s[N+i];

        /* The second half of the state vector is rate of change
         * of velocities, or the calculated accelerations.
         */
        sd[N+i] = qdd[i];
    }
    /* TODO: This should probably return the full sd instead of qdd.  What
     *       does OMPL want?
     */
    return sd;
}

void 
ckbot::chain_rate::tip_base_step(std::vector<double> s, std::vector<double> T)
{
    int N = c.num_links();
    
    std::vector<double> q(N);
    std::vector<double> qd(N);

    for(int i=0; i<N; ++i)
    {
        q[i] = s[i];
        qd[i] = s[N+i];
    }

    /* Declare and initialized all of the loop variables */
    Eigen::VectorXd grav(6);
    grav << 0,0,0,0,0,9.81; 
    Eigen::MatrixXd pp(6,6);
    pp = Eigen::MatrixXd::Zero(6,6);

    Eigen::VectorXd zp(6);
    zp << 0,0,0,0,0,0;

    Eigen::Matrix3d R_cur;
    Eigen::MatrixXd M_cur(6,6);

    Eigen::Vector3d r_i_ip(0,0,0);
    Eigen::Vector3d r_i_cm(0,0,0);

    Eigen::MatrixXd phi(6,6);
    Eigen::MatrixXd phi_cm(6,6);

    Eigen::MatrixXd p_cur(6,6);
    
    Eigen::VectorXd H_b_frame_star(6);
    Eigen::VectorXd H_w_frame_star(6);
    Eigen::RowVectorXd H(6);

    double D = 0.0;
    Eigen::VectorXd G(6);
    
    Eigen::MatrixXd tau_tilde(6,6);

    Eigen::Vector3d omega(0,0,0);
    Eigen::Matrix3d omega_cross;

    Eigen::VectorXd a(6);
    Eigen::VectorXd b(6);

    Eigen::VectorXd z(6);
    
    double C = 0.0;
    double epsilon = 0.0;

    double mu = 0.0;

    /* Recurse from the tip to the base of this chain */
    for (int i = N-1; i >= 0; --i)
    {
        module_link& cur = c.get_link(i);
        R_cur = c.get_current_R(i);
        r_i_ip = R_cur*(cur.get_r_ip1() - cur.get_r_im1());
        phi = get_body_trans(r_i_ip);
        
        r_i_cm = R_cur*(-cur.get_r_im1());

        phi_cm = get_body_trans(r_i_cm);

        M_cur = cur.get_spatial_inertia_mat();
        /* 
        std::cout << "phi':\n" << phi.transpose() << "\n";
        std::cout << "pp:\n" << pp << "\n";
        std::cout << "pp*phi':\n" << pp*phi.transpose() << "\n";
        */
        p_cur = phi*pp*phi.transpose() + M_cur;
        /* std::cout << "p_cur:\n" << p_cur << "\n"; */

        H_b_frame_star = cur.get_joint_matrix().transpose();

        Eigen::MatrixXd tmp_6x6(6,6);
        tmp_6x6 << R_cur, Eigen::Matrix3d::Zero(),
                   Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

        /* std::cout << "tmp 6x6: \n" << tmp_6x6 << "\n"; */
        H_w_frame_star = tmp_6x6*H_b_frame_star;
        
        H = H_w_frame_star.transpose();

        D = H*p_cur*H.transpose();
        G = p_cur*H.transpose()*(1.0/D);

        tau_tilde = Eigen::MatrixXd::Identity(6,6) - G*H;

        /* pp for the next time around the loop */
        pp = tau_tilde*p_cur;
        
        omega = c.get_angular_velocity(i);
        omega_cross = get_cross_mat(omega);

        // TODO: Use block matrix notiation and the stream (<<) operator here.
        b.topLeftCorner(3,1) = omega_cross*cur.get_I_cm()*omega;
        b.bottomLeftCorner(3,1) = cur.get_mass()*omega_cross*omega_cross*(-cur.get_r_im1());
        
        a.topLeftCorner(3,1) << 0,0,0;
        a.bottomLeftCorner(3,1) = omega_cross*omega_cross*(-cur.get_r_im1());
        
        z = phi*zp + p_cur*a + b + phi_cm*cur.get_mass()*grav;

        C = -cur.get_damping()*qd[i];

        epsilon = T[i] + C - H*z;

        mu = (1/D)*epsilon;
        zp = z + G*epsilon;

        mu_all[i] = mu;
        /*std::cout << "Filling the vectors needed by base to tip....\n";*/
        int cur_index=0;
        for (int k=(6*i); k <= (6*(i+1)-1); ++k,++cur_index)
        {
            /*std::cout << "k is: " << k << " cur_index is: " << cur_index << "\n";*/
            G_all[k] = G[cur_index];
            a_all[k] = a[cur_index];
        }
        /*
        std::cout << "M_cur:\n" << M_cur << "\n";
        std::cout << "phi:\n" << phi << "\n";
        std::cout << "r_i_cm:\n" << r_i_cm << "\n";
        std::cout << "R_cur:\n" << R_cur << "\n";
        std::cout << "H_w_frame_star:\n "<< H_w_frame_star << "\n";
        std::cout << "H: \n" << H << "\n";
        std::cout << "G: \n" << G << "\n";
        std::cout << "omega: \n" << omega << "\n";
        std::cout << "omega_cross: \n" << omega_cross << "\n";
        std::cout << "b: \n" << b << "\n";
        std::cout << "a: \n" << a << "\n";
        */
    }
}
/* Returns a vector of length N corresponding to each link's qdd */
std::vector<double>
ckbot::chain_rate::base_tip_step(std::vector<double> s, std::vector<double> T)
{
    int N = c.num_links();
    
    std::vector<double> q(N);
    std::vector<double> qd(N);
    std::vector<double> qdd(N);

    for(int i=0; i<N; ++i)
    {
        q[i] = s[i];
        qd[i] = s[N+i];
    }
    
    Eigen::Matrix3d R_cur;
    Eigen::Vector3d r_i_ip;
    Eigen::MatrixXd phi(6,6);
    Eigen::VectorXd alpha(6);
    alpha = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd alpha_p(6);
    Eigen::VectorXd G(6);
    Eigen::VectorXd a(6);
     
    Eigen::VectorXd H_b_frame_star(6);
    Eigen::VectorXd H_w_frame_star(6);
    Eigen::RowVectorXd H(6);
    /*std::cout << "Starting base to tip loop with " << N << " modules.\n";*/
    for (int i=0; i<N; ++i)
    {
        /*std::cout << "--- At link..." << i << "\n";*/
        module_link& cur = c.get_link(i);
        R_cur = c.get_current_R(i);
        r_i_ip = R_cur*(cur.get_r_ip1() - cur.get_r_im1());
        phi = get_body_trans(r_i_ip);
        
        alpha_p = phi.transpose()*alpha;

        int cur_index = 0;                
        for (int k = (6*i); k <= (6*(i+1)-1); ++k, ++cur_index)
        {
            /* std::cout << "k is: " << k << " cur_index is: " << cur_index << "\n";*/
            G[cur_index] = G_all[k];
            a[cur_index] = a_all[k];
        }

        qdd[i] = mu_all[i] - G.transpose()*alpha_p;

        H_b_frame_star = cur.get_joint_matrix().transpose();
        
        Eigen::MatrixXd tmp_6x6(6,6);
        tmp_6x6 << R_cur, Eigen::Matrix3d::Zero(),
                   Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

        H_w_frame_star = tmp_6x6*H_b_frame_star;

        H = H_w_frame_star.transpose();

        alpha = alpha_p + H.transpose()*qdd[i] + a;
        /*
        std::cout << "For link " << i << " alpha is: " << alpha << "\n";
        std::cout << "For link " << i << " G is: " << G << "\n";
        std::cout << "For link " << i << " a is: " << a << "\n";
        */
    }
    /* std::cout << "Exiting base_tip_step.\n"; */
    return qdd;
}

#endif
