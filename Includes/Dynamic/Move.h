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
	static void upDateVelocityContainer(int & Npl, int & Nplr, int & Nco, int & Nelb, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, double time, double dt, Gravity gt) noexcept;
	static void moveContainer(int & Npl, int & Nplr, int & Nco, int & Nelb, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, double time, double dt, std::vector<Sphere> & sph, Gravity gt) noexcept;
	static void upDateVelocitySphere(int & Nsph, std::vector<Sphere> & sph, Gravity gt, double dt) noexcept;
	static void upDateVelocitySphereOMP(int & Nsph, std::vector<Sphere> & sph, Gravity gt, double dt,int Nprocess) noexcept;
	static void moveSphere(int & Nsph, std::vector<Sphere> & sph, double dt) noexcept;
	static void moveSphereOMP(int & Nsph, std::vector<Sphere> & sph, double dt,int Nprocess) noexcept;
	static void moveBodies(int & Nbd, std::vector<Body> & bd, double dt, std::vector<Sphere> & sph) noexcept;
	static void moveBodiesOMP(int & Nbd, std::vector<Body> & bd, double dt, std::vector<Sphere> & sph,int Nprocess) noexcept;
	static void upDateVelocityBodies(int & Nbd, std::vector<Body> & bd, Gravity gt, double dt, std::vector<Sphere> & sph) noexcept;
	static void upDateVelocityBodiesOMP(int & Nbd, std::vector<Body> & bd, Gravity gt, double dt, std::vector<Sphere> & sph,int Nprocess) noexcept;
	static void MeltingSphere(int & Nsph, std::vector<Sphere> & sph, double vr, double delayVr, double dt) noexcept;
	static void upDateHollowBall(const int &Nhb, std::vector<HollowBall> & hb, double dt) noexcept;
};
