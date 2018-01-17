#pragma once

#include <iostream>
#include <vector>

class Sphere;
class Contact;
class Plan;
class PlanR;
class Body;
class Cone;
class Elbow;
class Data;
class HollowBall;
class Gravity;

class ContactDetection {
public:
	static int ContactSphSph(Sphere *a, Sphere *b, const double dila, Contact *ct, int & Nct) noexcept;
	static int ContactSphSph(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept;
	
	static void ContactSphPlan(Plan & p, Sphere *b, const double dila, Contact *ct, int & Nct) noexcept;
	static void ContactSphPlan(Plan & p, Sphere *b, Contact *ct, int & Nct) noexcept;

	static void ContactBodyPlan(Plan & p, Body *b, Contact *ct, int & Nct) noexcept;

	static void ContactSphPlanR(PlanR & p, Sphere *b, const double dila, Contact *ct, int & Nct) noexcept;
	static void ContactSphPlanR(PlanR & p, Sphere *b, Contact *ct, int & Nct) noexcept;

	static void ContactBodyPlanR(PlanR & p, Body *b, const double dila, Contact *ct, int & Nct) noexcept;
	static void ContactBodyPlanR(PlanR & p, Body *b, Contact *ct, int & Nct) noexcept;

	static void ContactSphCone(Cone & p, Sphere *b, const double dila, Contact *ct, int & Nct) noexcept;
	static void ContactSphCone(Cone & p, Sphere *b, Contact *ct, int & Nct) noexcept;
	static void ContactBodyCone(Cone & p, Body *b, const double dila, Contact *ct, int & Nct) noexcept;
	static void ContactBodyCone(Cone & p, Body *b, Contact *ct, int & Nct) noexcept;

	
	static void ContactSphElbow(Elbow & p, Sphere *b, const double dila, Contact *ct, int & Nct) noexcept;
	static void ContactSphElbow(Elbow & p, Sphere *b, Contact *ct, int & Nct) noexcept;
	static void ContactBodyElbow(Elbow & p, Body *b, const double dila, Contact *ct, int & Nct) noexcept;
	static void ContactBodyElbow(Elbow & p, Body *b, Contact *ct, int & Nct) noexcept;

	
	static int ContactSphBody(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept;
	static int ContactBodyBody(Body *a, Body *b, Contact *ct, int &Nct, double ra, double rb) noexcept;
	
	
	static int ContactSphBodyPeriodic(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept;
	static void ContactSphSphPeriodic(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept;
	static void ContactSphPlanPeriodic(std::vector<Sphere*>&llist, Plan & p, Plan & p2, Sphere *b, const double rmax) noexcept;
	
	static void sphContact(const int Nx0, const int Nx1, const int Nx, const int Ny0, const int Ny1, const int Ny,
												 const int Nz, Contact *ctl, int & Nctl, Sphere *cell[], const double dila) noexcept;
	
	static void sphContact(const int Nx0, const int Nx1, const int Nx, const int Ny0, const int Ny1, const int Ny,
												 const int Nz, Contact *ctl, int & Nctl, Sphere *cell[]) noexcept;
	
	static void sphContactAll(std::vector<Sphere> & sph, Contact *ctl, int & Nctl) noexcept;

	static void sphPlanContact(int & Nct, std::vector<Sphere> & sph,
														 std::vector<Plan> & pl, Contact *ct, Sphere *cell[], const double rmax) noexcept;
	
	static void sphPlanContactOMP(const int &Nsph, const int &Npl, int & Nct, int & Ncta, std::vector<Sphere> & sph,
																std::vector<Plan> & pl, Contact *ct, Contact *cta, Sphere *cell[], const double rmax) noexcept;
	
	static void sphPlanRContact(int & Nct, std::vector<PlanR> & plr, Contact *ct, Sphere *cell[]) noexcept;
	
	static void sphConeContact(int & Nct, std::vector<Cone> & co, Contact *ct, Sphere *cell[]) noexcept;
	
	static void sphElbowContact(int & Nct, std::vector<Elbow> & elb, Contact *ct, Sphere *cell[]) noexcept;
	
	static void sphHollowBallContact(int & Nct, std::vector<HollowBall> & hb, Contact *ct) noexcept;
	
	static void sphContainer(std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr,
													 std::vector<Cone> & co, std::vector<Elbow> & elb, int & Nct, Contact *ct, Sphere *cell[],
													 const double rmax) noexcept;
	static void sphContainer(std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, std::vector<HollowBall> & hb, int & Nct, Contact *ct, Sphere *cell[], const double rmax) noexcept;
	
	static void sphContainerOMP(const int & Nsph, const int & Npl, const int & Nplr, const int & Nco, const int & Nelb,
															const int & Nhb, std::vector<Sphere> & sph, std::vector<Plan> & pl,
															std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb,
															std::vector<HollowBall> & hb, int & Nct, Contact *ct, int & Ncta, Contact *cta,
															Sphere *cell[], const double rmax) noexcept;
	
	static void linkedCell(std::vector<Sphere> & sph, const Data *dat, Sphere *cell[]) noexcept;
	static void listCellForPlan(Data *dat, std::vector<Plan> & pl, int & Npl, double dila, Gravity& gt) noexcept;
	static void listCellForPlan(Data *dat, std::vector<Plan> & pl, int & Npl, Gravity& gt) noexcept;
	static void listCellForPlanR(Data *dat, std::vector<PlanR> & plr, int & Nplr, double dila, Gravity& gt) noexcept;
	static void listCellForPlanR(Data *dat, std::vector<PlanR> & plr, int & Nplr, Gravity& gt) noexcept;
	static void listCellForCone(Data *dat, std::vector<Cone> & co, int & Nco, double dila, Gravity& gt) noexcept;
	static void listCellForCone(Data *dat, std::vector<Cone> & co, int & Nco, Gravity& gt) noexcept;
	static void listCellForElbow(Data *dat, std::vector<Elbow> & elb, int & Nelb, double dila) noexcept;
	static void listCellForElbow(Data *dat, std::vector<Elbow> & elb, int & Nelb) noexcept;
};
