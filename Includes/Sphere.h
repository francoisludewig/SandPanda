#pragma once

#include "MechanicalPoint.h"
#include "ElongationManager.h"
#include <vector>

class Contact;
class ContactTampon;
class Data;
class Cell;
class Gravity;
class Body;
class Elongation;

class Sphere final : public MechanicalPoint{
private:
	static const int maxContact = 50;
	int num;
	double x2,y2,z2;
	int sp,bodies,NhollowBall;

	Sphere *tdl;
	Body *b;

	ElongationManager elongationManager;

	double r0;

	bool isHollowBall;
	bool autoIntegrate;
	double r,m,I,rho;
	//Contact avec plan
	double ct_pl_nx,ct_pl_ny,ct_pl_nz;
	int ct_pl;

public:
	Sphere() noexcept;
	Sphere(int bodies, int nHollowBall, double radius) noexcept;
	Sphere(double radius, double mass, double inertia) noexcept;
	~Sphere() noexcept;
	
	Sphere(const Sphere& other) = delete;
	Sphere(Sphere&& other) noexcept = default;
	Sphere& operator=(const Sphere& other) = delete;
	Sphere& operator=(Sphere&& other) noexcept = default;



	int Num() const noexcept { return num; }
	void Num(int rhs) noexcept { this->num = rhs; }

	double X2() const noexcept { return x2; }
	void X2(double rhs) noexcept { this->x2 = rhs; }
	double Y2() const noexcept { return y2; }
	void Y2(double rhs) noexcept { this->y2 = rhs; }
	double Z2() const noexcept { return z2; }
	void Z2(double rhs) noexcept { this->z2 = rhs; }

	int Bodies() const noexcept { return bodies; }

	double Radius() const noexcept { return r; }

	int HollowballNum() const noexcept { return NhollowBall; }
	double Rho() const noexcept { return rho; }
	double Mass() const noexcept { return m; }

	Sphere* TDL() const noexcept { return tdl; }
	void TDL(Sphere* rhs) noexcept { this->tdl = rhs; }

	Body* GetBody() const noexcept { return b; }
	void SetBody(Body* rhs) noexcept { this->b = rhs; }

	void EnableIntegration() noexcept { this->autoIntegrate = true; }
	void DisableIntegration() noexcept { this->autoIntegrate = false; }
    void IsHollowBall() noexcept { this->isHollowBall = true; }

    int Ct_pl() const noexcept { return ct_pl; }
    void Ct_pl(int rhs) noexcept { ct_pl = rhs; }

    int Ct_pl_nx() const noexcept { return ct_pl_nx; }
    void Ct_pl_nx(int rhs) noexcept { ct_pl_nx = rhs; }

    int Ct_pl_ny() const noexcept { return ct_pl_ny; }
    void Ct_pl_ny(int rhs) noexcept { ct_pl_ny = rhs; }

    int Ct_pl_nz() const noexcept { return ct_pl_nz; }
    void Ct_pl_nz(int rhs) noexcept { ct_pl_nz = rhs; }

	void affiche() const noexcept;
	void readFromFile(FILE *ft) noexcept;
	void writeToFile(FILE *ft) const noexcept;
	void writeOutFile(FILE *ft, int n, int mode) const noexcept;
	void readStartStop(FILE *ft) noexcept;
	
	void initTimeStep() noexcept;
	void Melt(double dt, double vr) noexcept;
	void Freeze(double dt, double vr) noexcept;
	void upDateVelocity(double dt, Gravity & g, double g0) noexcept;
	
	void move(double dt) noexcept;
	
	int NoBodies() const noexcept;
	int NoAvatar() const noexcept;
	void CancelVelocity() noexcept;
	void RandomVelocity(double V, double W) noexcept;
	
	bool Border() const noexcept;
	void ComputeCTD(double R, double w, double t) noexcept;
	void ComputeRD(double R, double w, double t) noexcept;
	int count() const noexcept;
	
	ElongationManager& GetElongationManager() { return elongationManager; }

	static void sphereLinking(int & Nsph , std::vector<Sphere> & sph,  std::vector<Body> & bd) noexcept;
};
