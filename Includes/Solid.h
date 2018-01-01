#pragma once

#include "Velocity.h"
#include "Gravity.h"
#include "MechanicalPoint.h"
#include <vector>

class Sphere;
class Contact;
class Fluid;

class Solid : public MechanicalPoint {
public:
	Solid() noexcept;
	~Solid() noexcept;
	
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
	double Vmax() noexcept;
	double Wmax() noexcept;
	double Delay() noexcept;
	void InitTimeStep() noexcept;
	void OnOffGravity(bool OnOff) noexcept;
	void SetVelocityToZero() noexcept;
	void SetMemoryPosition() noexcept;
	void GetMemoryPosition() noexcept;
	void UpdateForceFromGB(int & Nsph,std::vector<Sphere> & sph) noexcept;
	void UpdateGravityForceFromGB(int & Nsph,std::vector<Sphere> & sph, Gravity gt) noexcept;
	void SetVx(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetWx(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetVy(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetWy(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetVz(double newA0, double newA1, double newW, double newPhi) noexcept;
	void SetWz(double newA0, double newA1, double newW, double newPhi) noexcept;
	double ValueOfVx(double t) noexcept;
	double ValueOfVy(double t) noexcept;
	double ValueOfVz(double t) noexcept;
	double ValueOfWx(double t) noexcept;
	double ValueOfWy(double t) noexcept;
	double ValueOfWz(double t) noexcept;
	void UpDateVelocityLinkedSphere(std::vector<Sphere> & sph, double time) noexcept;
	void UpDateGravityVelocityLinkedSphere(std::vector<Sphere> & sph, double time, Gravity& gt) noexcept;
	void SetMass(double m) noexcept;
	void SetFcx(double fx) noexcept;
	void SetFcy(double fy) noexcept;
	void SetFcz(double fz) noexcept;
	double GetFcy() noexcept;
	void SetControlGB(int v,std::vector<Sphere> & sph) noexcept;
	
	int numero;
	// Base
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	double Fcx,Fcy,Fcz;
	double Mcx,Mcy,Mcz;
	double Mass,In,It,Is;
	int Force,activeGravity;
	Velocity V;
	// Linked cell
	int *Cell,NCell;
	int *Cell2,NCell2;
	// List of grain boder
	int *num;
	double *GBx,*GBy,*GBz;
	double R[3][3];
	double xMemory,yMemory,zMemory;
	int ControlGB;
	class Cell **vcell;
	int Ngb;
};
