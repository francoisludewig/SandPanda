#pragma once

#include "../Solids/Sphere.h"
#include "../Solids/Body.h"
#include "../Solids/Plan.h"
#include "../Solids/PlanR.h"
#include "../Solids/Cone.h"
#include "../Solids/Elbow.h"
#include "../Solids/BodySpecie.h"
#include "../Solids/HollowBall.h"
#include "../Configuration/Configuration.h"
#include "../Configuration/Gravity.h"

class SimulationData {
public:
	std::vector<Sphere> spheres;
	std::vector<Body> bodies;
	std::vector<Plan> plans;
	std::vector<PlanR> disks;
	std::vector<Cone> cones;
	std::vector<Elbow> elbows;
	std::vector<BodySpecie> bodySpecies;
	std::vector<HollowBall> hollowBalls;
	Gravity gravity;
	Configuration configuration;
	int sphereCount;
};
