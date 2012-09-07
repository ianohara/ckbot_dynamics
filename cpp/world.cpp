#include "World.hpp"
#include <cmath>

#include<iostream>

#define EPS 0.00001

Cylinder::Cylinder(Eigen::Vector3d Loc, Eigen::Vector3d Dir, double Len, double R) :
    loc(Loc),
    dir(Dir),
    len(Len)
{
    double dirL = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
    if (dirL - 1.0 > EPS) {
        std::cerr << "Warning, cylinder direction must be a unit vector.  Normalizing." << std::endl;
        dir = dir.normalized();
    }
}

Cylinder::Cylinder() :
    loc(Eigen::Vector3d::Zero()),
    dir(Eigen::Vector3d::UnitZ()),
    len(1.0)
{
}

Cylinder::~Cylinder()
{
}

Sphere::Sphere()
{
}

Sphere::Sphere(Eigen::Vector3d Loc, double Rad) :
    loc(Loc),
    r(Rad)
{
}

Sphere::~Sphere()
{
}

World::World(Cylinder* cs, unsigned int Num) :
    num(Num),
    cyls(Num)
{
    for (unsigned int i = 0; i < num; i++) {
        cyls[i] = cs[i];
    }
}

World::~World()
{
}

void
World::describe()
{
    std::cout << "The world of consists of: " << std::endl;
    for (unsigned int i = 0; i < num; i++) {
        std::cout << " Cyl " << i << ": " << std::endl;
        std::cout << "  loc = " << cyls[i].loc.transpose() << std::endl;
        std::cout << "  dir = " << cyls[i].dir.transpose() << std::endl;
        std::cout << "  len = " << cyls[i].len << std::endl;
        std::cout << "  r   = " << cyls[i].r << std::endl;
    }
}

boost::shared_ptr<World>
get_world(const Json::Value& jroot)
{

    if (!jroot["cylinders"].isArray())
    {
        std::cerr << "In get_world: world json must have a root dictionary key named 'cylinders'." << std::endl;
        return boost::shared_ptr<World>();
    }
    unsigned int num = jroot["cylinders"].size();

    Cylinder cyls[num];
    Eigen::Vector3d loc;
    Eigen::Vector3d dir;
    double len, r;

    for (unsigned int i = 0; i < num; i++) {
        for (unsigned int j = 0; j < 3; j++) {
            loc[j] = jroot["cylinders"][i]["loc"][j].asDouble();
            dir[j] = jroot["cylinders"][i]["dir"][j].asDouble();
        }
        len = jroot["cylinders"][i]["len"].asDouble();
        r = jroot["cylinders"][i]["r"].asDouble();
        cyls[i].loc = loc;
        cyls[i].dir = dir;
        cyls[i].len = len;
        cyls[i].r = r;
    }
    return boost::shared_ptr<World>(new World(cyls, num));
}

/* For each sphere, check and see if it collides with any of the world's
 * cylinders
 */
bool
World::checkForCollisions(const std::vector<Sphere> &sphs)
{
    Eigen::Vector3d pHat, cHat, nHat, r_s, r_c, r_cs, r_ns, r_nc;
    double L, Rc, Rs;
    for (unsigned int sid = 0; sid < sphs.size(); sid++)
    {
        for (unsigned int cid = 0; cid < cyls.size(); cid++)
        {
            cHat = cyls[cid].dir;
            L = cyls[cid].len;
            Rc = cyls[cid].r;
            Rs = sphs[sid].r;
            r_c = cyls[cid].loc;
            r_s = sphs[sid].loc;
            r_cs = r_s  - r_c;
            pHat = (cHat.cross(r_s - r_c)).normalized();
            nHat = cHat.cross(pHat);
            r_ns = abs(r_cs.dot(nHat))*nHat;
            r_nc = r_s - r_ns;

            /* See if normal line between cylinder axis and
             * sphere center falls between the start and
             * end of the cylinder
             */
            if (((r_c - r_nc).dot(cHat) > 0.0) /* Before start*/ 
                || (((r_c + L*cHat) - r_nc).dot(cHat) < 0.0)) /* After end */
            {
                return true;
            }

            /* We know now that the normal line between the two falls
             * somewhere in the cylinder.  Check to see if there is intersection
             */
            if (r_ns.norm() < (Rs + Rc)) {
                return false;
            }
        }
    }
    return true;
}
