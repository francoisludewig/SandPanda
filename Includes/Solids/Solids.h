#pragma once

#include "Sphere.h"
#include "Body.h"
#include "Plan.h"
#include "PlanR.h"
#include "Cone.h"
#include "Elbow.h"
#include "BodySpecie.h"
#include "HollowBall.h"
#include "../Configuration.h"
#include "../Gravity.h"

class Solids {
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
