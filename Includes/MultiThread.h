#pragma once

#include <vector>
#include <thread>

#include "Configuration/Configuration.h"
#include "Dynamic/Evolution.h"

#include "Solids/Plan.h"
#include "Solids/PlanR.h"
#include "Solids/Cone.h"
#include "Solids/Elbow.h"
#include "Solids/Sphere.h"
#include "Solids/HollowBall.h"
#include "LinkedCells/CellBounds.h"

class MultiThread {
public:
	MultiThread(int threadCount, int sphereCount, int bodyCount, std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb, Configuration & dat, Gravity & gf, const CellBounds& cellBounds);

	void Run(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co,std::vector<Elbow> & elb,std::vector<Sphere> & sph,std::vector<Body> & bd,std::vector<HollowBall> & hb,Configuration & dat,Gravity & gf,
    		std::vector<Sphere*>& cell, int & Ntp, char *name, int Nthreshold);
private:
	std::vector<std::thread> threads;
	std::vector<Evolution> evolutions;
};
