#pragma once

#include "MechanicalPoint.h"
#include <vector>

class Contact;
class Data;
class Cell;
class Gravity;
class Body;
class Elongation;

class Sphere: public MechanicalPoint{
public:
	static const int maxContact = 50;
	
	int num;
	double x2,y2,z2;
	int sp,bodies,NhollowBall;
	double r,m,I,rho;
	double r0;
	bool isHollowBall;
	Body *b;
	bool autoIntegrate;
	Sphere *tdl;
	// Data contact statique
    std::vector<Elongation> xsi, tp_xsi;
    std::vector<int> neighbourNumber, tp_neighbourNumber;
    std::vector<int> type, tp_type;
    std::vector<int> bodyNumber, tp_bodyNumber;
	int Nneighbour,Nneighbour2;

    //Contact avec plan
	double ct_pl_nx,ct_pl_ny,ct_pl_nz;
	int ct_pl;
	
public:
	Sphere() noexcept;
	~Sphere() noexcept;
	
	void SphDealloc() noexcept;
	int HollowballNum() const noexcept;
	double Radius() const noexcept;
	double Rho() const noexcept;
	double Mass() const noexcept;
	void affiche() const noexcept;
	void readFromFile(FILE *ft) noexcept;
	void writeToFile(FILE *ft) const noexcept;
	void writeOutFile(FILE *ft, int n, int mode) const noexcept;
	void readStartStop(FILE *ft) noexcept;
	
	void Melt(double dt, double vr) noexcept;
	void Freeze(double dt, double vr) noexcept;
	void upDateVelocity(double dt, Gravity & g, double g0) noexcept;
	
	int Num() const noexcept;
	void move(double dt) noexcept override;
	
	void InitXsi() noexcept;
	void AddXsi(Elongation& e, int n, int t, int nob) noexcept;
	Elongation FoundIt(int n, int t, int nob) const noexcept;
	double radius() const noexcept;
	void setRadius(double alpha) noexcept;
	int NoBodies() const noexcept;
	int NoAvatar() const noexcept;
	void RandomVelocity(double V, double W) noexcept;
	
	bool Border() const noexcept;
	double getFx() const noexcept;
	double getFy() const noexcept;
	double getFz() const noexcept;
	double getRho() const noexcept;
	void setIsHollowBall(bool a) noexcept;
	void ComputeCTD(double R, double w, double t) noexcept;
	void ComputeRD(double R, double w, double t) noexcept;
	int count() const noexcept;
	
	static void sphereLinking(int & Nsph , std::vector<Sphere> & sph,  std::vector<Body> & bd) noexcept;
};
