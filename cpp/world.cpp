#include "World.hpp"
#include <cmath>

#include<iostream>

#define EPS 0.00001

Cylinder::Cylinder(Eigen::Vector3d &Loc, Eigen::Vector3d &Dir, double Len, double R) :
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
    std::cout << "Destroying Cyl..." << std::endl;
}

const void
Cylinder::describe(void) const
{
    std::cout << "  Cylinder Desc:" << std::endl;
    std::cout << " Woot:" << loc << std::endl;
    std::cout << "  loc = " << loc.transpose() << std::endl;
    std::cout << "  dir = " << dir.transpose() << std::endl;
    std::cout << "  len = " << len << std::endl;
    std::cout << "  r   = " << r << std::endl;
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
    std::cout << "Destroying Sph..." << std::endl;
}

const void
Sphere::describe(void) const
{
    std::cout << "  loc = " << loc.transpose() << std::endl;
    std::cout << "  r   = " << r << std::endl;
}

World::World(Cylinder cs[], unsigned int Num) :
    num(Num),
    cyls(Num)
{
    for (unsigned int i = 0; i < num; i++) {
        cyls[i].loc = cs[i].loc;
        cyls[i].dir = cs[i].dir;
        cyls[i].r = cs[i].r;
        cyls[i].len = cs[i].len;
    }
    std::cout << "Printing cyls in constructor..." << std::endl;
    for (unsigned int j=0; j < cyls.size(); j++) {
        std::cout << "Cyls[" << j << "].loc:" << cyls[j].loc << std::endl;
    }
    std::cout << "Done!" << std::endl;
}

World::~World()
{
    std::cout << "Destroying world..." << std::endl;
}

void
World::describe()
{
    std::cout << "The world of consists of: " << std::endl;
    for (unsigned int i = 0; i < num; i++) {
        std::cout << " Cyl " << i << ": " << std::endl;
        cyls[i].describe();
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

/* For a sphere, check and see if it collides with any of the world's
 * cylinders
 */
bool
World::isColliding(Sphere sph)
{
    std::cout << "Entering checkForCollisions" << std::endl; //DEBUG
    Eigen::Vector3d pHat; //cHat, nHat, r_s, r_c, r_cs, r_ns, r_nc;
    Eigen::Vector3d cHat;
    Eigen::Vector3d nHat;
    Eigen::Vector3d r_s;
    Eigen::Vector3d r_c;
    Eigen::Vector3d r_cs;
    Eigen::Vector3d r_ns;
    Eigen::Vector3d r_nc;
    double L=0, Rc=0, Rs=0;
    std::cout << "cyls size is: " << cyls.size() << std::endl;
    for (int cid = 0; cid < cyls.size(); cid++)
    {
        cyls[cid].describe();
        std::cout << "  cid = " << cid << std::endl; //DEBUG
        cHat = cyls[cid].dir;
        std::cout << "  cHat = " << cHat.transpose() << std::endl;
        L = cyls[cid].len;
        std::cout << "  cyl Len=" << L << std::endl;
        Rc = cyls[cid].r;
        Rs = sph.r;
        r_c = cyls[cid].loc;
        std::cout << "  r_c=" << r_c.transpose() << std::endl;
        r_s = sph.loc;
        std::cout << "  r_s=" << r_s.transpose() << std::endl;
        r_cs = r_s  - r_c;
        std::cout << "  r_cs=" << r_cs.transpose() << std::endl;
        pHat = (cHat.cross(r_s - r_c)).normalized();
        std::cout << "  pHat=" << pHat.transpose() << std::endl;
        nHat = cHat.cross(pHat);
        std::cout << "  nHat=" << nHat.transpose() << std::endl;
        r_ns = r_cs.dot(nHat)*nHat;
        std::cout << "  r_ns=" << r_ns.transpose() << std::endl;
        r_nc = r_s - r_ns;
        std::cout << "  r_nc=" << r_nc.transpose() << std::endl;

        /* See if normal line between cylinder axis and
         * sphere center falls between the start and
         * end of the cylinder
         */
        if (((r_c - r_nc).dot(cHat) > 0.0) /* Before start*/ 
            || (((r_c + L*cHat) - r_nc).dot(cHat) < 0.0)) /* After end */
        {
            continue;
        }

        /* We know now that the normal line between the two falls
         * somewhere in the cylinder.  Check to see if there is intersection
         */
        if (r_ns.norm() < (Rs + Rc)) {
            std::cout << "Sphere:" << std::endl;
            sph.describe();
            std::cout << "Cylinder:" << std::endl;
            cyls[cid].describe();
            std::cout << "    Distance is: " << r_ns.norm() << " colliding dist is " << (Rs + Rc) << std::endl;
            return true;
        }
    }
    std::cout << "Not colliding..." << std::endl;
    return false;
}
