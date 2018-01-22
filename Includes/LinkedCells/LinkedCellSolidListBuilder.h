#pragma once

#include <vector>

class Data;
class Plan;
class PlanR;
class Cone;
class Elbow;
class Gravity;

class LinkedCellSolidListBuilder{
public:
	static void ListCellForPlan(Data *dat, std::vector<Plan> & pl, Gravity& gt) noexcept;
	static void ListCellForPlanR(Data *dat, std::vector<PlanR> & plr, Gravity& gt) noexcept;
	static void ListCellForCone(Data *dat, std::vector<Cone> & co, Gravity& gt) noexcept;
	static void ListCellForElbow(Data *dat, std::vector<Elbow> & elb) noexcept;
};
