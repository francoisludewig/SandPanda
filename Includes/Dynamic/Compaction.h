#pragma once

#include <vector>
#include <memory>

class Plan;
class PlanR;
class Cone;
class Elbow;
class Sphere;
class Configuration;
class Body;
class HollowBall;
class Contact;
class Gravity;
class CellBounds;
class SimulationData;

class Compaction {
public:
	static void Secousse(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, double Gamma, double f, Configuration & dat) noexcept;
	
	static void Relaxation(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Configuration & dat) noexcept;
	
	static int Run(std::shared_ptr<SimulationData>& solids, std::vector<Sphere*>& cell, int & Ntp, char *name,int ntpi, int ntpf, double Gamma, double f, int Nthreshold, const CellBounds& cellBounds) noexcept;
};
