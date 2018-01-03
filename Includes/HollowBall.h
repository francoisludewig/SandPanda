#pragma once

#include <iostream>
#include <vector>

using namespace std;

class Sphere;
class Contact;

class HollowBall{
    Sphere **inSph,**cell,*avatar;
    int Navatar;
    int NinSph;
    double x,y,z,q0,q1,q2,q3;
    double vx,vy,vz,wx,wy,wz;
    bool lockVx,lockVy,lockVz;
    bool lockWx,lockWy,lockWz;
    double mass,Inertia,r;
    double xmin,ymin,zmin;
    double a,rmax;
    int N,*list,Nlist;
public:
    HollowBall() noexcept;
    ~HollowBall() noexcept;
    void loadFromFile(FILE *ft) noexcept;
    void writeToFile(FILE *ft) const noexcept;
    void LinkInSph(vector<Sphere> & sph, int & Nsph, const int numero) noexcept;
    void ContactDetectionInHollowBall(Contact *ct, int & Nct) noexcept;
    void ContactDetectionWithHollowBall(Contact *ct, int & Nct) noexcept;
    void UpdateFromSph(double dt) noexcept;
    void Makeavatar(std::vector<Sphere> & sph, int & Nsph, const int numero) noexcept;
};
