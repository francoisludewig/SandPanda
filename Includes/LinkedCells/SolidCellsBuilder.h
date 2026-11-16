#pragma once

#include <vector>
#include "SolidCells.h"

class Configuration;
class Plan;
class PlanR;
class Cone;
class Elbow;
class Gravity;
class CellBounds;

class SolidCellsBuilder{
public:

	static SolidCells Build(const Configuration& dat, std::vector<Plan>& pl, std::vector<PlanR>& plr,
			    std::vector<Cone>& co, std::vector<Elbow>& elb, Gravity& gt, const CellBounds& cellBounds);

private:
	static void ListCellForPlan(SolidCells& solidCells, const Configuration& dat, std::vector<Plan> & pl, Gravity& gt, const CellBounds& cellBounds) noexcept;
	static void ListCellForPlanR(SolidCells& solidCells, const Configuration& dat, std::vector<PlanR> & plr, Gravity& gt, const CellBounds& cellBounds) noexcept;
	static void ListCellForCone(SolidCells& solidCells, const Configuration& dat, std::vector<Cone> & co, Gravity& gt, const CellBounds& cellBounds) noexcept;
	static void ListCellForElbow(SolidCells& solidCells, const Configuration& dat, std::vector<Elbow> & elb, const CellBounds& cellBounds) noexcept;
};
