#pragma once

class MechanicalPoint {
public:
	MechanicalPoint() noexcept :
	x(0), y(0), z(0),
	q0(1), q1(0), q2(0), q3(0),
	vx(0), vy(0), vz(0),
	wx(0), wy(0), wz(0),
	Fx(0), Fy(0), Fz(0),
	Mx(0), My(0), Mz(0) {}
	
	MechanicalPoint(const MechanicalPoint& other) noexcept = default;
	MechanicalPoint(MechanicalPoint&& other) noexcept = default;
	MechanicalPoint& operator=(const MechanicalPoint& other) noexcept = default;
	MechanicalPoint& operator=(MechanicalPoint&& other) noexcept = default;
	
	double x, y, z;
	double q0, q1, q2, q3;
	double vx, vy, vz;
	double wx, wy, wz;
	double Fx, Fy, Fz;
	double Mx, My, Mz;
};
