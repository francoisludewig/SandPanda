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
	static int ContactSphSph(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept;
	static void ContactSphPlan(Plan & p, Sphere *b, Contact *ct, int & Nct) noexcept;
	static void ContactBodyPlan(Plan & p, Body *b, Contact *ct, int & Nct) noexcept;
	static void ContactSphPlanR(PlanR & p, Sphere *b, Contact *ct, int & Nct) noexcept;
	static void ContactBodyPlanR(PlanR & p, Body *b, Contact *ct, int & Nct) noexcept;
	static void ContactSphCone(Cone & p, Sphere *b, Contact *ct, int & Nct) noexcept;
	static void ContactBodyCone(Cone & p, Body *b, Contact *ct, int & Nct) noexcept;
	static void ContactSphElbow(Elbow & p, Sphere *b, Contact *ct, int & Nct) noexcept;
	static void ContactBodyElbow(Elbow & p, Body *b, Contact *ct, int & Nct) noexcept;
	static int ContactSphBody(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept;
	static int ContactBodyBody(Body *a, Body *b, Contact *ct, int &Nct, double ra, double rb) noexcept;
	static int ContactSphBodyPeriodic(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept;
	static void ContactSphSphPeriodic(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept;
	static void ContactSphPlanPeriodic(std::vector<Sphere*>&llist, Plan & p, Plan & p2, Sphere *b, const double rmax) noexcept;
	
	
	static void sphContact(const int Nx0, const int Nx1, const int Nx, const int Ny0, const int Ny1, const int Ny,
												 const int Nz, Contact *ctl, int & Nctl, std::vector<Sphere*>& cell) noexcept;
	
	static void sphContactAll(std::vector<Sphere> & sph, Contact *ctl, int & Nctl) noexcept;
	static void sphPlanContact(int & Nct, std::vector<Sphere> & sph,
														 std::vector<Plan> & pl, Contact *ct, std::vector<Sphere*>& cell, const double rmax) noexcept;
	static void sphPlanRContact(int & Nct, std::vector<PlanR> & plr, Contact *ct, std::vector<Sphere*>& cell) noexcept;
	static void sphConeContact(int & Nct, std::vector<Cone> & co, Contact *ct, std::vector<Sphere*>& cell) noexcept;
	static void sphElbowContact(int & Nct, std::vector<Elbow> & elb, Contact *ct, std::vector<Sphere*>& cell) noexcept;
	static void sphHollowBallContact(int & Nct, std::vector<HollowBall> & hb, Contact *ct) noexcept;
	static void sphContainer(std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, std::vector<HollowBall> & hb, int & Nct, Contact *ct, std::vector<Sphere*>& cell, const double rmax) noexcept;
};
