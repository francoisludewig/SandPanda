#pragma once

#include <iostream>
#include <vector>
#include "Plan.h"
#include "PlanR.h"
#include "Cone.h"
#include "Elbow.h"

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
	// TODO Use std::vector<double>
	double *dxpl,*dypl,*dzpl;
	// TODO Use std::vector<Plan*>
	Plan **pl;
	// TODO Use std::vector<double>
	double *dxplr,*dyplr,*dzplr;
	// TODO Use std::vector<PlanR*>
	PlanR **plr;
	// TODO Use std::vector<double>
	double *dxco,*dyco,*dzco;
	// TODO Use std::vector<Cone*>
	Cone **co;
public:
	MasterSolid() noexcept;
	void initPlan(int a) noexcept;
	void initPlanR(int a) noexcept;
	void initCone(int a) noexcept;
	
	void addPlan(std::vector<Plan> & pl2, int n, int m) noexcept;
	void addPlanR(std::vector<PlanR> & plr2, int n, int m) noexcept;
	void addCone(std::vector<Cone> & co2, int n, int m) noexcept;
	void ComputePara() noexcept;
	void getForces() noexcept;
	void UpDateVelocity(double h) noexcept;
	void Move(double h) noexcept;
	void UpDateSolid() noexcept;
};

