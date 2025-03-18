#pragma once

#include "Velocity.h"
#include "../Configuration/Gravity.h"
#include "MechanicalPoint.h"
#include <vector>

class Sphere;
class Contact;
class Fluid;

class Solid : public MechanicalPoint {
public:
	Solid() noexcept;
	~Solid() noexcept;
	
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
	void move(double dt) noexcept override;
	void MoveGravity(double dt, Gravity& gt) noexcept;
	[[nodiscard]] double Vmax() const noexcept;
	[[nodiscard]] double Wmax() const noexcept;
	[[nodiscard]] double Delay() const noexcept;
	void OnOffGravity(bool OnOff) noexcept;
	void SetMemoryPosition() noexcept;
	void GetMemoryPosition() noexcept;
	void SetVz(double newA0, double newA1, double newW, double newPhi) noexcept;

	int numero{};
	// Base
    double x0, y0, z0;
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	double Fcx,Fcy,Fcz;
	double Mcx,Mcy,Mcz;
	double Mass,In,It,Is;
    double xMemory, yMemory, zMemory;
	int Force,activeGravity;
	Velocity V;
	// Linked cell
	int *Cell,NCell;
};
