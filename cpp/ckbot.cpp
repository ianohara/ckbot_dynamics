#include<vector>
#include<eigen3/Eigen/Dense>
#include<iostream>

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

    class module_link
    {
    public:
        module_link():
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
        module_link(double damping, 
                Eigen::Vector3d forward_joint_axis,
                Eigen::Vector3d r_im1,
                Eigen::Vector3d r_ip1,
                Eigen::Matrix3d I_cm,
                Eigen::Matrix3d R_jts,
                Eigen::Matrix3d init_rotation,
                double m) : 
            q_(0.0),
            qd_(0.0),
            damping_(damping),
            forward_joint_axis_(forward_joint_axis),
            r_im1_(r_im1),
            r_ip1_(r_ip1),
            I_cm_(I_cm),
            R_jts_(R_jts),
            init_rotation_(init_rotation),
            m_(m) 
        {
        }
        module_link(struct module_description m): 
            q_(0.0),
            qd_(0.0),
            damping_(m.damping),
            forward_joint_axis_(m.forward_joint_axis),
            r_im1_(m.r_im1),
            r_ip1_(m.r_ip1),
            I_cm_(m.I_cm),
            R_jts_(m.R_jts),
            init_rotation_(m.init_rotation),
            m_(m.m)
        {
        }
        module_link(const module_link& source):
            q_(source.get_q()),
            qd_(source.get_qd()),
            damping_(source.get_damping()),
            forward_joint_axis_(source.get_forward_joint_axis()),
            r_im1_(source.get_r_im1()),
            r_ip1_(source.get_r_ip1()),
            I_cm_(source.get_I_cm()),
            R_jts_(source.get_R_jts()),
            init_rotation_(source.get_init_rotation()),
            m_(source.get_mass())
        {
        }
        module_link& operator=(module_link& source)
        {
            damping_ = source.get_damping();
            forward_joint_axis_ = source.get_forward_joint_axis();
            r_im1_ = source.get_r_im1();
            r_ip1_ = source.get_r_ip1();
            I_cm_ = source.get_I_cm();
            R_jts_ = source.get_R_jts();
            init_rotation_ = source.get_init_rotation();
            m_ = source.get_mass();
        }
        virtual ~module_link(void)
        {
        }
        double get_q(void) const
        {
            return q_;
        }
        double get_qd(void) const
        {
            return qd_;
        }
        Eigen::Vector3d get_forward_joint_axis(void) const
        {
            return forward_joint_axis_;
        }
        Eigen::Vector3d get_r_im1(void) const
        {
            return r_im1_;
        }
        Eigen::Vector3d get_r_ip1(void) const
        {
            return r_ip1_;
        }
        Eigen::Matrix3d get_I_cm(void) const
        {
            return I_cm_;
        }
        Eigen::Matrix3d get_R_jts(void) const
        {
            return R_jts_;
        }
        Eigen::Matrix3d get_init_rotation(void) const
        {
            return init_rotation_;
        }
        double get_damping(void) const
        {
            return damping_;
        }
        double get_mass(void) const
        {
            return m_;
        }
        void set_q(double q)
        {
            q_ = q;
        }
        void set_qd(double qd)
        {
            qd_ = qd;
        }
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
        chain(int N) : N_(N), links_(N)
        { 
        }
        chain(module_link chain_modules[], int num_modules) : N_(num_modules), links_(num_modules)
        {
            for (int link=0; link<num_modules; ++link)
            {
                links_[link] = chain_modules[link];
            }
        }
        ~chain(void)
        {
        }
        int num_links(void)
        {
            return N_;
        }
        void propogate_angles_and_rates(std::vector<double> q, std::vector<double> qd)
        {
            for (int i=0; i<N_; ++i)
            {
                links_[i].set_q(q[i]);
                links_[i].set_qd(qd[i]);
            }
        }
        void print_masses(void)
        {
            std::vector<module_link>::iterator link_iterator;
            int num = 0;
            for(link_iterator = links_.begin(); link_iterator != links_.end(); link_iterator++)
            {
                std::cout << "Link " << ++num << " has mass of " << link_iterator->get_mass() << " [kg]\n";
            }
        }
    };
    class chain_rate
    {
    protected:
        std::vector<double> G_all;
        std::vector<double> mu_all;
        std::vector<double> a_all;
        std::vector<double> sd;

        chain &c;
    public:
        chain_rate(chain& ch) : 
            c(ch), 
            G_all(ch.num_links()),
            mu_all(ch.num_links()),
            a_all(ch.num_links()),
            sd(2*ch.num_links())
        {
        }
        /* Given a state vector of length 2*N corresponding to N joint angles
         * and N joint speeds and torque vector of length N, 
         * this fills in the 2*N length vector sd with
         * the corresponding joint speeds and joint accelerations.
         */
        std::vector<double> calc_rate(std::vector<double> s, std::vector<double> T)
        {
            
        }
        void tip_base_step(std::vector<double> s)
        {
            Eigen::VectorXd grav(6);
            grav << 0,0,0,0,0,9.81; 
            Eigen::VectorXd pp(6);
            pp << 0,0,0,0,0,0;
            Eigen::VectorXd zp(6);
            zp << 0,0,0,0,0,0;

            int N = c.num_links();
            
            std::vector<double> q(N);
            std::vector<double> qd(N);
            
            for(int i=0; i<N; ++i)
            {
                q[i] = s[i];
                qd[i] = s[N+i];
            }

            c.propogate_angles_and_rates(q,qd);

            /* Recurse from the tip to the base of this chain */
            for (int i = N-1; i >= 0; --i)
            {
                std::cout << "At chain link " << i << " in the tip to base step.\n";
            }
        }
        /* Returns a vector of length N corresponding to each link's qdd */
        std::vector<double> base_tip_step(void)
        {
        
        }
    };
}

int main(void)
{
    double damping = 0.5;
    Eigen::Vector3d forward_joint_axis(0,0,1);
    Eigen::Vector3d r_im1(0,0,1);
    Eigen::Vector3d r_ip1(0,1,0);
    Eigen::Matrix3d I_cm;
    Eigen::Matrix3d R_jts;
    Eigen::Matrix3d init_rotation;
    double m = 1.0;

    I_cm << 1,0,0,
         0,1,0,
         0,0,1;
    
    R_jts << 1,0,0,
         0,1,0,
         0,0,1;

    init_rotation << 1,0,0,
         0,1,0,
         0,0,1;

    struct ckbot::module_description HT1 = {damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, init_rotation, 2.0}; 

    ckbot::module_link ck = ckbot::module_link(damping, forward_joint_axis, r_im1, r_ip1, I_cm, R_jts, init_rotation, m);

    ckbot::module_link ck2 = ckbot::module_link(HT1);

    std::cout << "The Joint angle is: " << ck.get_q() << "\n";
    std::cout << "The forward joint axis is: \n" << ck.get_forward_joint_axis() << "\n";

    std::cout << "The Joint angle for ck2 is is: " << ck2.get_q() << "\n";
    std::cout << "The forward joint axis is: \n" << ck2.get_forward_joint_axis() << "\n";


    ckbot::module_link chain_modules[] = {ck, ck2};
    int num_modules = 2;
    ckbot::chain ch = ckbot::chain(chain_modules, num_modules);
    ch.print_masses();

    ckbot::chain_rate rate_machine(ch);
    std::vector<double> s0(2*num_modules);
    std::fill(s0.begin(), s0.end(), 0.0);

    rate_machine.tip_base_step(s0);

    return 0;
}
