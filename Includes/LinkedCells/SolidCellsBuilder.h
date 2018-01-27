#pragma once

#include <vector>
#include "SolidCells.h"

class Data;
class Plan;
class PlanR;
class Cone;
class Elbow;
class Gravity;

class SolidCellsBuilder{
public:

	static SolidCells Build(const Data& dat, std::vector<Plan>& pl, std::vector<PlanR>& plr,
			    std::vector<Cone>& co, std::vector<Elbow>& elb, Gravity& gt);

private:
	static void ListCellForPlan(SolidCells& solidCells, const Data& dat, std::vector<Plan> & pl, Gravity& gt) noexcept;
	static void ListCellForPlanR(SolidCells& solidCells, const Data& dat, std::vector<PlanR> & plr, Gravity& gt) noexcept;
	static void ListCellForCone(SolidCells& solidCells, const Data& dat, std::vector<Cone> & co, Gravity& gt) noexcept;
	static void ListCellForElbow(SolidCells& solidCells, const Data& dat, std::vector<Elbow> & elb) noexcept;
};
