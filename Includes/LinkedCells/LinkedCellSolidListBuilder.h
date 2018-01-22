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
	static void ListCellForPlan(const Data& dat, std::vector<Plan> & pl, Gravity& gt) noexcept;
	static void ListCellForPlanR(const Data& dat, std::vector<PlanR> & plr, Gravity& gt) noexcept;
	static void ListCellForCone(const Data& dat, std::vector<Cone> & co, Gravity& gt) noexcept;
	static void ListCellForElbow(const Data& dat, std::vector<Elbow> & elb) noexcept;
};
