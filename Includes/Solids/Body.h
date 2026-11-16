#pragma once

#include "MechanicalPoint.h"
#include "../Elongations/Elongation.h"
#include "../Elongations/ElongationManager.h"
#include <vector>
#include <array>
#include <tuple>
#include <cstdio>

class Sphere;
class Contact;
class Plan;
class PlanR;
class Fluid;
class Configuration;
class Cone;
class Elbow;
class BodySpecie;
class Gravity;

class Body : public MechanicalPointWithBase{
public:
	Body();
	~Body();

	[[nodiscard]] int NumberOfSphere() const noexcept { return Ng; }
	[[nodiscard]] int Num() const noexcept { return numl; }
	[[nodiscard]] double GetRmax() const noexcept { return Rmax; }

	void LoadFromFile(FILE *ft) noexcept;
	void ReadStartStopFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft,std::vector<Sphere> & sph) const noexcept;
	void WriteOutFile(FILE *ft, int mode) const noexcept;
	void TimeStepInitialization() noexcept;
	void UpDateVelocity(double dt, const Gravity & g) noexcept;
	void Move(double dt) noexcept;
	void UpDateLinkedSphere(std::vector<Sphere> & sph) noexcept;
	void UpDateLinkedSphereTp() noexcept;
	void UploadSpecies(std::vector<BodySpecie> bdsp, std::vector<Sphere> & sph, int numero) noexcept;
	void CancelVelocity() noexcept;
	void RandomVelocity(double V, double W) noexcept;
	void SetActiveRotation(int na) noexcept;
	
	[[nodiscard]] int SphereCount() const noexcept { return Ng; }
	[[nodiscard]] double SphereX(const int i) const noexcept { return xg[i]; }
	[[nodiscard]] double SphereY(const int i) const noexcept { return yg[i]; }
	[[nodiscard]] double SphereZ(const int i) const noexcept { return zg[i]; }
	[[nodiscard]] double SphereRadius(const int i) const noexcept { return r[i]; }
	[[nodiscard]] double Mass() const noexcept { return m; }
    [[nodiscard]] double MaximumRadius() const noexcept { return Rmax; }

	ElongationManager& GetElongationManager() { return elongationManager; }

	[[nodiscard]] std::tuple<double, double, double> Lever(const double& cnx, const double& cny, const double& cnz, const int& na) const noexcept {
		return std::make_tuple( r[na]*cnx + (xg[na] - x),
													  r[na]*cny + (yg[na] - y),
													  r[na]*cnz + (zg[na] - z));
	}

private:
	int sp;
	int Ng;
	int numl;
	double Rmax;
	double m;
	double Ine_1[3][3];
	int NhollowBall;
	std::vector<double> xl;
	std::vector<double> yl;
	std::vector<double> zl;
	std::vector<double> r;
	std::vector<double> xg;
	std::vector<double> yg;
	std::vector<double> zg;
	ElongationManager elongationManager{250};

	int ActiveRotation;
};

