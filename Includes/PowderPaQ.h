#pragma once

#include <vector>

class Plan;
class PlanR;
class Cone;
class Elbow;
class Sphere;
class HollowBall;
class Data;
class Gravity;
class Sphere;
class Body;
class Data;
class Contact;
class CellBounds;

class PowderPaQ {
public:
	static int PowderPaQRun(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Data & dat,Gravity & gf,
			std::vector<Sphere*>& cell, int & Ntp, char *name,int ntpi, int ntpf, int Nthreshold, double PQheight, double PQVel, const CellBounds& cellBounds) noexcept;
private:
	static void PowderPaQsecousseUpward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Data & dat, double PQheight, double PQVel) noexcept;
	static void PowderPaQsecousseDownward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Data & dat, double PQheight) noexcept;
	static void PowderPaQrelaxation(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Data & dat, double t, double PQheight, double PQVel) noexcept;

};
