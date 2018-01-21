#pragma once

#include <vector>

class Plan;
class PlanR;
class Cone;
class Elbow;
class Sphere;
class Data;
class Body;
class HollowBall;
class Contact;
class Gravity;

class Compaction {
public:
	static void Secousse(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, double Gamma, double f, Data & dat) noexcept;
	
	static void Relaxation(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Data & dat) noexcept;
	
	static int Run(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Data & dat,Gravity & gf,
								 Sphere *cell[], int & Ntp, char *name,bool record,int ntpi, int ntpf, double Gamma, double f, int Nthreshold) noexcept;
};
