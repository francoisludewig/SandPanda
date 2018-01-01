#pragma once

#include "BodySpecie.h"
#include "Gravity.h"
#include "Elongation.h"
#include "HollowBall.h"
#include "ComputingForce.h"
#include "MechanicalPoint.h"
#include "ContactDetection.h"
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

class Body : public MechanicalPoint{
public:
	Body() noexcept;
	~Body() noexcept;
	void LoadFromFile(FILE *ft) noexcept;
	void ReadStartStopFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft,vector<Sphere> & sph) const noexcept;
	void WriteOutFile(FILE *ft, int mode) const noexcept;
	void TimeStepInitialization() noexcept;
	void UpDateVelocity(double dt, Gravity & g) noexcept;
	void Move(double dt) noexcept;
	void UpDateLinkedSphere(vector<Sphere> & sph) noexcept;
	void UpDateLinkedSphereTp() noexcept;
	void UploadSpecies(int Nbdsp, vector<BodySpecie> bdsp, vector<Sphere> & sph, int & Nsph, int numero) noexcept;
	int NumberOfSphere() const noexcept;
	void CancelVelocity() noexcept;
	void RandomVelocity(double V, double W) noexcept;
	int Num() const noexcept;
	double GetRmax() const noexcept;
	void InitXsi() noexcept;
	void AddXsi(Elongation e, int n, int t, int selfn, int nob) noexcept;
	Elongation FoundIt(int n, int t, int selfn, int nob) const noexcept;
	void SetActiveRotation(int na) noexcept;
	
public:
	int sp;
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
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
	
private:
	inline void QuaternionToBase() noexcept;
	
};

