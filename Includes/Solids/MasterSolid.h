#pragma once

#include <iostream>
#include <vector>

class Plan;
class PlanR;
class Cone;
class Elbow;

class MasterSolid{
	// Position of center of mass
	double x,y,z;
	// Velocity
	double vx,vy,vz;
	// Acceleration
	double Fx,Fy,Fz;
	double Fcx,Fcy,Fcz;
	double Mass;
	
	int Npl,Nplr,Nco;

	std::vector<double> dxpl;
	std::vector<double> dypl;
	std::vector<double> dzpl;
	std::vector<Plan*> pl;

	std::vector<double> dxplr;
	std::vector<double> dyplr;
	std::vector<double> dzplr;
	std::vector<PlanR*> plr;

	std::vector<double> dxco;
	std::vector<double> dyco;
	std::vector<double> dzco;
	std::vector<Cone*> co;
public:
	MasterSolid() noexcept;
	
	void addPlan(std::vector<Plan> & pl2, int n, int m) noexcept;
	void addPlanR(std::vector<PlanR> & plr2, int n, int m) noexcept;
	void addCone(std::vector<Cone> & co2, int n, int m) noexcept;
	void ComputePara() noexcept;
	void getForces() noexcept;
	void UpDateVelocity(double h) noexcept;
	void Move(double h) noexcept;
	void UpDateSolid() noexcept;
};

