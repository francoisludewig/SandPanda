#pragma once

#include "Velocity.h"
#include "../Configuration/Gravity.h"
#include "MechanicalPoint.h"
#include <vector>

class Sphere;
class Contact;
class Fluid;

class Solid : public MechanicalPointWithBase {
public:
	Solid() noexcept;
	~Solid() noexcept;
	
	[[nodiscard]] double ValueOfVx(const double t) const noexcept { return(V.ValueOfVx(t)); }
	[[nodiscard]] double ValueOfVy(const double t) const noexcept { return(V.ValueOfVy(t)); }
	[[nodiscard]] double ValueOfVz(const double t) const noexcept { return(V.ValueOfVz(t));	}
	[[nodiscard]] double ValueOfWx(const double t) const noexcept { return(V.ValueOfWx(t)); }
	[[nodiscard]] double ValueOfWy(const double t) const noexcept { return(V.ValueOfWy(t)); }
	[[nodiscard]] double ValueOfWz(const double t) const noexcept { return(V.ValueOfWz(t)); }
	void SetMass(const double m) noexcept { Mass = m; }

	class Cell** getVcell() noexcept;
	void LoadFromFile(FILE *ft) noexcept;
	void LoadAccelerationFromFile(FILE *ft) noexcept;
	void WriteAccelerationFromFile(FILE *ft) const noexcept;
	void WriteToFile(FILE *ft) const noexcept;
	void WriteOutFile(FILE *ft, int mode) const noexcept;
	void ComputeBase() noexcept;
	void Display() const noexcept;
	void UpDateForce() noexcept;
	void UpDateVelocity(double time, double dt, Gravity& gt) noexcept;
	void UpDateGravityVelocity(double time, double dt, Gravity& gt) noexcept;
	void Move(double dt) noexcept;
	void MoveGravity(double dt, Gravity& gt) noexcept;
	void UpDateLinkedSphere(std::vector<Sphere> & sph, double time, Gravity& gt) noexcept;
	void UpDateGravityLinkedSphere(std::vector<Sphere> & sph, double time, Gravity& gt) noexcept;
	[[nodiscard]] double Vmax() noexcept;
	[[nodiscard]] double Wmax() noexcept;
	double Delay() noexcept;
	void TimeStepInitialization() noexcept;
	void OnOffGravity(bool OnOff) noexcept;
	void SetVelocityToZero() noexcept;
	void SetMemoryPosition() noexcept;
	void GetMemoryPosition() noexcept;
	void UpdateForceFromGB(std::vector<Sphere> & sph) noexcept;
	void UpdateGravityForceFromGB(int & Nsph,std::vector<Sphere> & sph, Gravity gt) noexcept;
	void SetVx(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetWx(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetVy(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetWy(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetVz(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetWz(double newA0, double newA1, double newW, double newPhi) noexcept;
	void UpDateVelocityLinkedSphere(std::vector<Sphere> & sph, double time) noexcept;
	void UpDateGravityVelocityLinkedSphere(std::vector<Sphere> & sph, double time, Gravity& gt) noexcept;
	void SetFcx(double fx) noexcept;
	void SetFcy(double fy) noexcept;
	void SetFcz(double fz) noexcept;
	double GetFcy() noexcept;
	void SetControlGB(int v,std::vector<Sphere> & sph) noexcept;
	

	[[nodiscard]] double GetMass() const noexcept { return Mass; }
	[[nodiscard]] double GetFcx() const noexcept { return Fcx; }
	[[nodiscard]] double GetFcy() const noexcept { return Fcy; }
	[[nodiscard]] double GetFcz() const noexcept { return Fcz; }

	[[nodiscard]] const Velocity& GetV() const noexcept { return V; }
	[[nodiscard]] int Numero() const noexcept { return numero; }
	void Numero(const int rhs) noexcept { numero = rhs; }

	[[nodiscard]] int GetForce() const noexcept { return Force; }
	void SetForce(const int rhs) noexcept { Force = rhs; }

	[[nodiscard]] int GetNgb() const noexcept { return Ngb; }

	// Linked cell
	//int *Cell,NCell;

protected:
	int numero;
	// Base
	double Fcx,Fcy,Fcz;
	double Mcx,Mcy,Mcz;
	double Mass,In,It,Is;
	int Force,activeGravity;
	Velocity V;
	// List of grain boder
	int *num;
	double *GBx,*GBy,*GBz;
	double R[3][3];
	double xMemory,yMemory,zMemory;
	int ControlGB;
	class Cell **vcell;
	int Ngb;
};
