#pragma once

#include <iostream>
#include <vector>

class Plan;
class PlanR;
class Cone;
class Elbow;
class Gravity;
class Sphere;
class Body;
class HollowBall;

class Move {
public:
	static void upDateVelocityContainer(std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, double time, double dt, Gravity gt) noexcept;
	static void moveContainer(std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, double time, double dt, std::vector<Sphere> & sph, Gravity gt) noexcept;
	static void upDateVelocitySphere(std::vector<Sphere> & sph, Gravity gt, double dt) noexcept;
	static void upDateVelocitySphereOMP(int & Nsph, std::vector<Sphere> & sph, Gravity gt, double dt,int Nprocess) noexcept;
	static void moveSphere(std::vector<Sphere> & sph, double dt) noexcept;
	static void moveBodies(std::vector<Body> & bd, double dt, std::vector<Sphere> & sph) noexcept;
	static void upDateVelocityBodies(std::vector<Body> & bd, Gravity gt, double dt, std::vector<Sphere> & sph) noexcept;
	static void upDateVelocityBodiesOMP(int & Nbd, std::vector<Body> & bd, Gravity gt, double dt, std::vector<Sphere> & sph,int Nprocess) noexcept;
	static void MeltingSphere(std::vector<Sphere> & sph, double vr, double delayVr, double dt) noexcept;
	static void UpDateForceContainer(std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, double time, double dt, Gravity gt) noexcept;
	static void upDateVelocityLinkedSphereContainer(std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, double time, double dt, std::vector<Sphere> & sph) noexcept;
	static void upDateHollowBall(std::vector<HollowBall> & hb, double dt) noexcept;
};
