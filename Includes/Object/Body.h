#pragma once

#include "BodySpecie.h"
#include "../Configuration/Gravity.h"
#include "../Contact/Elongation.h"
#include "../Contact/ElongationManager.h"
#include "HollowBall.h"
#include "../Contact/ComputingForce.h"
#include "MechanicalPoint.h"
#include "../Contact/ContactDetection.h"
#include "../Contact/ContactIdentifier.h"
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
    static const int maxContact = 250;

    Body() noexcept;
	~Body() noexcept;
	void LoadFromFile(FILE *ft) noexcept;
	void ReadStartStopFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft) const noexcept;
	void WriteOutFile(FILE *ft, int mode) const noexcept;
	void UpDateVelocity(double dt, Gravity & g) noexcept;
	void move(double dt) noexcept override;
	void UpDateLinkedSphere(vector<Sphere> & sph) noexcept;
	void UpDateLinkedSphereTp() noexcept;
	void UploadSpecies(vector<BodySpecie> & bdsp, vector<Sphere> & sph, int & Nsph, int numero) noexcept;
	int NumberOfSphere() const noexcept;
	void RandomVelocity(double V, double W) noexcept;
	int Num() const noexcept;
	double GetRmax() const noexcept;
	void InitXsi() noexcept;
	//void AddXsi(Elongation e, int n, int t, int selfn, int nob) noexcept;
	//Elongation FoundIt(int n, int t, int selfn, int nob) const noexcept;

    void AddXsi(Elongation& e, uint64_t contactIdentifier) noexcept;
    Elongation FoundIt(uint64_t contactIdentifier) const noexcept;

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

    ElongationManager elongationManager{maxContact};
/*
    std::array<Elongation, maxContact> xsi;
	std::array<int, maxContact> NumNeighbour;
	std::array<int, maxContact> type;
	std::array<int, maxContact> NumFromBody;
	std::array<int, maxContact> SelfNumFromBody;
	int Nneighbour,Nneighbour2;
 */
	int ActiveRotation;
	
private:
	inline void QuaternionToBase() noexcept;
	
};

