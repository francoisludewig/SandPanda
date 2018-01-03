#pragma once

#include "MechanicalPoint.h"
#include "Elongation.h"
#include <vector>
#include <array>

class Sphere;
class Contact;
class Plan;
class PlanR;
class Fluid;
class Data;
class Cone;
class Elbow;
class BodySpecie;
class Gravity;

class Body : public MechanicalPointWithBase{
public:
	Body() noexcept;
	~Body() noexcept;

	int NumberOfSphere() const noexcept { return Ng; }
	int Num() const noexcept { return numl; }
	double GetRmax() const noexcept { return Rmax; }

	void LoadFromFile(FILE *ft) noexcept;
	void ReadStartStopFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft,std::vector<Sphere> & sph) const noexcept;
	void WriteOutFile(FILE *ft, int mode) const noexcept;
	void TimeStepInitialization() noexcept;
	void UpDateVelocity(double dt, Gravity & g) noexcept;
	void Move(double dt) noexcept;
	void UpDateLinkedSphere(std::vector<Sphere> & sph) noexcept;
	void UpDateLinkedSphereTp() noexcept;
	void UploadSpecies(int Nbdsp, std::vector<BodySpecie> bdsp, std::vector<Sphere> & sph, int & Nsph, int numero) noexcept;
	void CancelVelocity() noexcept;
	void RandomVelocity(double V, double W) noexcept;
	void InitXsi() noexcept;
	void AddXsi(Elongation e, int n, int t, int selfn, int nob) noexcept;
	Elongation FoundIt(int n, int t, int selfn, int nob) const noexcept;
	void SetActiveRotation(int na) noexcept;
	
public:
	int sp;
	int Ng;
	int numl;
	double Rmax;
	double m;
	double Ine_1[3][3];
	double cx,cy,cz;
	int NhollowBall;
	std::vector<double> xl;
	std::vector<double> yl;
	std::vector<double> zl;
	std::vector<double> r;
	std::vector<double> xg;
	std::vector<double> yg;
	std::vector<double> zg;
	std::array<Elongation, 250> xsi;
	std::array<int, 250> NumNeighbour;
	std::array<int, 250> type;
	std::array<int, 250> NumFromBody;
	std::array<int, 250> SelfNumFromBody;
	int Nneighbour,Nneighbour2;
	int ActiveRotation;
};

