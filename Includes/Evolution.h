#pragma once

#include "Contact.h"

#include <vector>

class Sphere;
class Plan;
class PlanR;
class Cone;
class Elbow;
class Body;
class HollowBall;
class Data;
class Gravity;


class Evolution{
public:
	Evolution(int sphereCount, int bodyCount) noexcept {
		ct = new Contact[18*sphereCount+75*bodyCount];
		Nct = 0;
	}

	~Evolution() noexcept { delete[] ct;}

	Contact* GetContacts() noexcept { return ct; }
	int ContactCount() const noexcept { return Nct; }

    int Evolve(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Data & dat,Gravity & gf,
								Sphere *cell[], int & Ntp, char *name,bool record, int Nthreshold) noexcept;
	
    int EvolveMelt(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Data & dat,Gravity & gf,
										Sphere *cell[], int & Ntp, char *name,bool record, double vr, double delayVr, int Nthreshold) noexcept;

private:
	Contact *ct;
    int Nct;
};
