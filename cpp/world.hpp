#ifndef _WORLD_HPP
#define _WORLD_HPP

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <json/json.h>
#include <vector>

class Cylinder
{
    public:
        Cylinder(Eigen::Vector3d Loc, Eigen::Vector3d Dir, double Len, double R);
        Cylinder();
        ~Cylinder();
        Eigen::Vector3d loc;
        Eigen::Vector3d dir;
        double len;
        double r;
        const void describe(void) const;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Sphere
{
    public:
        Sphere();
        Sphere(Eigen::Vector3d Loc, double Rad);
        ~Sphere();
        Eigen::Vector3d loc;
        double r;
        const void describe(void) const;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class World
{
    public:
        World(std::vector<Cylinder> &cs, unsigned int Num);
        ~World();

        void describe();

        bool isColliding(Sphere &sph);
        unsigned int num;
        std::vector<Cylinder> cyls;
};

bool
isColliding(Sphere &sph);

boost::shared_ptr<World>
get_world(const Json::Value& jroot);

#endif /* _WORLD_HPP */
