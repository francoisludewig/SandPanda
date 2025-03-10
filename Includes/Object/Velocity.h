#pragma once

#include "Sinusoid.h"

class Velocity{
public:
	enum class VelocityType{
		vx,
		vy,
		vz,
		wx,
		wy,
		wz
	};
	
	Velocity() noexcept;
	Velocity(const Velocity& other) noexcept = default;
	Velocity(Velocity&& other) noexcept = default;
	Velocity& operator=(const Velocity& other) noexcept = default;
	Velocity& operator=(Velocity&& other) noexcept = default;
	
	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft, int mode) const noexcept;
	void LoadFromFileWithoutOrigin(FILE *ft) noexcept;
	void WriteToFileWithoutOrigin(FILE *ft, int mode) const noexcept;
	
	void Set(VelocityType type, double a0, double a1, double w, double phi) noexcept;
	
	double ValueOfVx(double t) const noexcept;
	double ValueOfVy(double t) const noexcept;
	double ValueOfVz(double t) const noexcept;
	double ValueOfWx(double t) const noexcept;
	double ValueOfWy(double t) const noexcept;
	double ValueOfWz(double t) const noexcept;
	double VMax() const noexcept;
	double WMax() const noexcept;
	double Delay() const noexcept;
	void Display() const noexcept;
	
	double ox,oy,oz;
private:
	Sinusoid vx,vy,vz,wx,wy,wz;
};
