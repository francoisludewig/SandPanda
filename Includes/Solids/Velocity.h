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
	
	[[nodiscard]] double ValueOfVx(const double t) const noexcept { return vx.Value(t); }
	[[nodiscard]] double ValueOfVy(const double t) const noexcept { return vy.Value(t); }
	[[nodiscard]] double ValueOfVz(const double t) const noexcept { return vz.Value(t); }
	[[nodiscard]] double ValueOfWx(const double t) const noexcept { return wx.Value(t); }
	[[nodiscard]] double ValueOfWy(const double t) const noexcept { return wy.Value(t); }
	[[nodiscard]] double ValueOfWz(const double t) const noexcept { return wz.Value(t); }



	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft, int mode) const noexcept;
	void LoadFromFileWithoutOrigin(FILE *ft) noexcept;
	void WriteToFileWithoutOrigin(FILE *ft, int mode) const noexcept;
	
	void Set(VelocityType type, double a0, double a1, double w, double phi) noexcept;
	[[nodiscard]] double VMax() const noexcept;
	[[nodiscard]] double WMax() const noexcept;
	[[nodiscard]] double Delay() const noexcept;
	void Display() const noexcept;
	
	double ox,oy,oz;
private:
	Sinusoid vx,vy,vz,wx,wy,wz;
};
