#pragma once

#include <vector>

class Sphere;
class Plan;
class PlanR;
class Cone;
class Elbow;
class Body;
class HollowBall;
class Contact;
class Data;
class Gravity;

class Evolution{
public:
	static int Evolve(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,
										std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
								Sphere *cell[], int & Ntp, char *name,bool record, int Nthreshold, bool isMonitoringActivated) noexcept;
	
	static int EvolveOMP(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,int & Ncta,int & Nctb,int & Nctc,
									 std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Contact *ct,Contact *cta,Contact *ctb,Contact *ctc,Data & dat,Gravity & gf,
									 Sphere *cell[], int & Ntp, char *name,bool record, int Nthreshold, bool isMonitoringActivated) noexcept;
	
	static int EvolveMelt(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,
										std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Contact *ct,Data & dat,Gravity & gf,
										Sphere *cell[], int & Ntp, char *name,bool record, double vr, double delayVr, int Nthreshold) noexcept;
	
	static int EvolveMeltOMP(int & Npl,int & Nplr,int & Nco,int & Nelb,int & Nsph,int & Nsph0,int & Nbd,int & Nhb,int & Nct,int & Ncta,int & Nctb,int & Nctc,
											 std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Contact *ct,Contact *cta,Contact *ctb,Contact *ctc,Data & dat,Gravity & gf,
											 Sphere *cell[], int & Ntp, char *name,bool record, double vr, double delayVr, int Nthreshold) noexcept;
};
