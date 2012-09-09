#include "World.hpp"
#include <cmath>
#include<unistd.h>

#include<iostream>

#define EPS 0.00001

Cylinder::Cylinder(Eigen::Vector3d Loc, Eigen::Vector3d Dir, double Len, double R) 
{
    if (Dir.norm() - 1.0 > EPS) {
        std::cerr << "Warning, cylinder direction must be a unit vector.  Normalizing." << std::endl;
    }
    loc << Loc;
    dir << Dir.normalized();
    r = R;
    len = Len;
}

Cylinder::Cylinder() :
    loc(Eigen::Vector3d::Zero()),
    dir(Eigen::Vector3d::UnitZ()),
    r(1),
    len(1)
{
}

Cylinder::~Cylinder()
{
}

const void
Cylinder::describe(void) const
{
    std::cout << "  Cylinder Desc:" << std::endl;
    std::cout << "    len = " << len << std::endl;
    std::cout << "    r   = " << r << std::endl;
    std::cout << "    loc = " << loc << std::endl;
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

const void
Sphere::describe(void) const
{
    std::cout << "  Sphere Desc: " << std::endl;
    std::cout << "    loc = " << loc.transpose() << std::endl;
    std::cout << "    r   = " << r << std::endl;
}

World::World(std::vector<Cylinder> &cs, unsigned int Num) :
    num(Num)
{
    for (unsigned int i = 0; i < cs.size(); i++) {
        cyls.push_back(cs[i]);
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
        std::cout << " " << i << ": " << std::endl;
        cyls[i].describe();
    }
}

boost::shared_ptr<World>
get_world(const Json::Value& jroot)
{

    if (!jroot["cylinders"].isArray())
    {
        std::cerr << "In get_world: world json must have a root dictionary "
                  << "key named 'cylinders'." << std::endl;
        return boost::shared_ptr<World>();
    }
    unsigned int num = jroot["cylinders"].size();

    std::vector<Cylinder> cyls;
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

        Cylinder tmpC(loc, dir, len, r);
        cyls.push_back(tmpC);
    }
    return boost::shared_ptr<World>(new World(cyls, num));

}

/* For a sphere, check and see if it collides with any of the world's
 * cylinders
 */
bool
World::isColliding(Sphere &sph)
{
    Eigen::Vector3d pHat, cHat, nHat, r_s, r_c, r_cs, r_ns, r_nc;
    double L=0, Rc=0, Rs=0;
    for (int cid = 0; cid < cyls.size(); cid++)
    {
        cHat = cyls[cid].dir; /* Cylinder direction unit vector */
        L = cyls[cid].len; /* Cylinder length */
        Rc = cyls[cid].r;  /* Cylinder radius */
        Rs = sph.r; /* Sphere radius */
        /* Location of base of cylinder (extends length L in cHat
         * direction from here) */
        r_c = cyls[cid].loc;
        r_s = sph.loc; /* Location of center of sphere */
        r_cs = r_s  - r_c; /* Vector from cylinder base to sphere center */
        /* Unit vector pointing out of the plane defined by cHat and r_cs */
        pHat = (cHat.cross(r_s - r_c)).normalized();
        /* Unit vector normal to cHat and in the plane intersecting the
         * sphere center and the cylinder's center line. */
        nHat = cHat.cross(pHat);
        /* Vector from sphere center to sphere center line contact point such
         * that r_ns is normal to cHat */
        r_ns = r_cs.dot(nHat)*nHat;
        /* Vector from origin to intersection of r_ns with cylinder center line */
        r_nc = r_s - r_ns;
//DEBUG        std::cout << "Collision Check Summary:" << std::endl
//DEBUG           << "  Sphere centered at: " << r_s.transpose() << " with radius: " << Rs
//DEBUG           << std::endl << "  Cylinder with base at: " << r_c.transpose()
//DEBUG           << " with len: " << L << " and radius: " << Rc << std::endl
//DEBUG           << "  Normal distance is: " << r_ns.norm() << std::endl;
//DEBUG
//DEBUG        std::cout << "  r_nc=" << r_nc.transpose() << std::endl;
//DEBUG        std::cout << "  r_ns=" << r_ns.transpose() << std::endl;
//DEBUG        std::cout << "  nHat=" << nHat.transpose() << std::endl;
//DEBUG        std::cout << "  pHat=" << pHat.transpose() << std::endl;
//DEBUG        std::cout << "  r_cs=" << r_cs.transpose() << std::endl;
//DEBUG        std::cout << "  r_s=" << r_s.transpose() << std::endl;
//DEBUG        std::cout << "  cyl Len=" << L << std::endl;
//DEBUG        std::cout << "  cHat = " << cHat.transpose() << std::endl;
//DEBUG        std::cout << "  cid = " << cid << std::endl; //DEBUG
//DEBUG        std::cout << "  r_c=" << r_c.transpose() << std::endl;
//DEBUG

        /* See if normal line between cylinder axis and
         * sphere center falls between the start and
         * end of the cylinder
         */
        if (((r_c - r_nc).dot(cHat) > 0.0) /* Before start*/ 
            || (((r_c + L*cHat) - r_nc).dot(cHat) < 0.0)) /* After end */
        {
//DEBUG            std::cout << "  Cannot collide! Normal line is before or after end of cyl." << std::endl;
            continue;
        }

        /* We know now that the normal line between the two falls
         * somewhere in the cylinder.  Check to see if there is intersection
         */
        if (r_ns.norm() < (Rs + Rc)) {
//DEBUG std::cout << "    Distance is: " << r_ns.norm() << " colliding dist is " << (Rs + Rc) << std::endl;
            return true;
        }
    }
    return false;
}

