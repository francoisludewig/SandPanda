#pragma once

#include <vector>

class Sphere;
class Plan;
class PlanR;
class Cone;
class Elbow;
class Body;
class HollowBall;
class Data;
class Contact;
class Gravity;


class Evolution{
public:
	static int Evolve(int & Nct, std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
								Sphere *cell[], int & Ntp, char *name,bool record, int Nthreshold) noexcept;
	
	static int EvolveMelt(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,
										std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
										Sphere *cell[], int & Ntp, char *name,bool record, double vr, double delayVr, int Nthreshold) noexcept;
};
