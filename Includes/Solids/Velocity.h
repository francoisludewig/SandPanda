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
	
	double ValueOfVx(double t) const noexcept { return vx.Value(t); }
	double ValueOfVy(double t) const noexcept { return vy.Value(t); }
	double ValueOfVz(double t) const noexcept { return vz.Value(t); }
	double ValueOfWx(double t) const noexcept { return wx.Value(t); }
	double ValueOfWy(double t) const noexcept { return wy.Value(t); }
	double ValueOfWz(double t) const noexcept { return wz.Value(t); }



	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft, int mode) const noexcept;
	void LoadFromFileWithoutOrigin(FILE *ft) noexcept;
	void WriteToFileWithoutOrigin(FILE *ft, int mode) const noexcept;
	
	void Set(VelocityType type, double a0, double a1, double w, double phi) noexcept;
	double VMax() const noexcept;
	double WMax() const noexcept;
	double Delay() const noexcept;
	void Display() const noexcept;
	
	double ox,oy,oz;
private:
	Sinusoid vx,vy,vz,wx,wy,wz;
};
