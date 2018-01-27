#pragma once

#include "Contact/Contact.h"

#include "LinkedCells/SolidCells.h"
#include "LinkedCells/Cells.h"
#include <vector>
#include "LinkedCells/SolidCellsBuilder.h"

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
	Evolution(int sphereCount, int bodyCount, std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb, Data & dat, Gravity & gf) noexcept {
		ct = new Contact[18*sphereCount+75*bodyCount];
		solidCells = SolidCellsBuilder::Build(dat, pl, plr, co, elb, gf);
		Nct = 0;
	}

	~Evolution() noexcept { delete[] ct;}

	Contact* GetContacts() noexcept { return ct; }
	int ContactCount() const noexcept { return Nct; }

    int Evolve(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Data & dat,Gravity & gf,
    		std::vector<Sphere*>& cell, int & Ntp, char *name, int Nthreshold) noexcept;
	
    int EvolveMelt(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Data & dat,Gravity & gf,
    		std::vector<Sphere*> cell, int & Ntp, char *name, double vr, double delayVr, int Nthreshold) noexcept;

private:
	Contact *ct;
	SolidCells solidCells;
    int Nct;
};
