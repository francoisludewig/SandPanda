#pragma once

#include <vector>
#include <memory>

class Plan;
class PlanR;
class Cone;
class Elbow;
class Sphere;
class HollowBall;
class Configuration;
class Gravity;
class Sphere;
class Body;
class Configuration;
class Contact;
class CellBounds;
class SimulationData;

class PowderPaQ {
public:
	static int PowderPaQRun(std::shared_ptr<SimulationData>& solids, std::vector<Sphere*>& cell, int & Ntp, char *name, int ntpi, int ntpf, double PQheight, double PQVel, const CellBounds& cellBounds, bool
	                        isMonitoringActivated) noexcept;
private:
	static void PowderPaQsecousseUpward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Configuration & dat, double PQheight, double PQVel) noexcept;
	static void PowderPaQsecousseDownward(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Configuration & dat, double PQheight) noexcept;
	static void PowderPaQrelaxation(std::vector<Plan> & pl,std::vector<PlanR> & plr,std::vector<Cone> & co, Configuration & dat, double t, double PQheight, double PQVel) noexcept;

};
