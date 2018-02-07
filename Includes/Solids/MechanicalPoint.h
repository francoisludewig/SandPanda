#pragma once

#include <mutex>

class MechanicalPoint {
public:
	MechanicalPoint() noexcept :
	x(0), y(0), z(0),
	q0(1), q1(0), q2(0), q3(0),
	vx(0), vy(0), vz(0),
	wx(0), wy(0), wz(0),
	Fx(0), Fy(0), Fz(0),
	Mx(0), My(0), Mz(0) {
		forceMutex = std::unique_ptr<std::mutex>(new std::mutex());
		momentumMutex = std::unique_ptr<std::mutex>(new std::mutex());
	}

	MechanicalPoint(const MechanicalPoint& other) = delete;
	MechanicalPoint(MechanicalPoint&& other) = default;
	MechanicalPoint& operator=(const MechanicalPoint& other) = delete;
	MechanicalPoint& operator=(MechanicalPoint&& other) = default;

	const double& X() const noexcept { return x; }
	const double& Y() const noexcept { return y; }
	const double& Z() const noexcept { return z; }
	
	void X(const double& lhs) noexcept { this->x = lhs; }
	void Y(const double& lhs) noexcept { this->y = lhs; }
	void Z(const double& lhs) noexcept { this->z = lhs; }
	
	void Q0(const double& lhs) noexcept { this->q0 = lhs; }
	void Q1(const double& lhs) noexcept { this->q1 = lhs; }
	void Q2(const double& lhs) noexcept { this->q2 = lhs; }
	void Q3(const double& lhs) noexcept { this->q3 = lhs; }
	
	const double& Vx() const noexcept { return vx; }
	const double& Vy() const noexcept { return vy; }
	const double& Vz() const noexcept { return vz; }
	
	void Vx(const double& lhs) noexcept { this->vx = lhs; }
	void Vy(const double& lhs) noexcept { this->vy = lhs; }
	void Vz(const double& lhs) noexcept { this->vz = lhs; }
	

	const double& Wx() const noexcept { return wx; }
	const double& Wy() const noexcept { return wy; }
	const double& Wz() const noexcept { return wz; }

	void Wx(const double& lhs) noexcept { this->wx = lhs; }
	void Wy(const double& lhs) noexcept { this->wy = lhs; }
	void Wz(const double& lhs) noexcept { this->wz = lhs; }

	const double& GetFx() const noexcept { return Fx; }
	const double& GetFy() const noexcept { return Fy; }
	const double& GetFz() const noexcept { return Fz; }


	const double& GetMx() const noexcept { return Mx; }
	const double& GetMy() const noexcept { return My; }
	const double& GetMz() const noexcept { return Mz; }

	void AddForce(const double fx, const double fy, const double fz) noexcept {
		Fx += fx;
		Fy += fy;
		Fz += fz;
	}
	
	void AddMomemtum(const double mx, const double my, const double mz) noexcept {
		Mx += mx;
		My += my;
		Mz += mz;
	}

	void AddForceMutex(const double fx, const double fy, const double fz) noexcept {
		std::lock_guard<std::mutex> lock(*forceMutex);
		Fx += fx;
		Fy += fy;
		Fz += fz;
	}

	void AddMomemtumMutex(const double mx, const double my, const double mz) noexcept {
		std::lock_guard<std::mutex> lock(*momentumMutex);
		Mx += mx;
		My += my;
		Mz += mz;
	}

	void PointVelocityMutex(double& vpx, double& vpy, double& vpz,
			const double& lx, const double& ly, const double& lz) const noexcept{
		vpx = vx + wy*lz - wz*ly;
		vpy = vy + wz*lx - wx*lz;
		vpz = vz + wx*ly - wy*lx;
	}
	
	
	void PointVelocity(double& vpx, double& vpy, double& vpz,
										 const double& lx, const double& ly, const double& lz) const noexcept{
		vpx = vx + wy*lz - wz*ly;
		vpy = vy + wz*lx - wx*lz;
		vpz = vz + wx*ly - wy*lx;
	}

protected:
	double x, y, z;	
	double q0, q1, q2, q3;
	double vx, vy, vz;
	double wx, wy, wz;
	double Fx, Fy, Fz;
	double Mx, My, Mz;
	std::unique_ptr<std::mutex> forceMutex {nullptr};
	std::unique_ptr<std::mutex> momentumMutex {nullptr};
};


class MechanicalPointWithBase : public MechanicalPoint {

public:
	MechanicalPointWithBase() noexcept : MechanicalPoint(),
	nx(1), ny(0), nz(0),
	tx(0), ty(1), tz(0),
	sx(0), sy(0), sz(1) {}

	MechanicalPointWithBase(const MechanicalPointWithBase& other) = delete;
	MechanicalPointWithBase(MechanicalPointWithBase&& other) = default;
	MechanicalPointWithBase& operator=(const MechanicalPointWithBase& other) = delete;
	MechanicalPointWithBase& operator=(MechanicalPointWithBase&& other) = default;

	double Nx() const noexcept { return nx; }
	double Ny() const noexcept { return ny; }
	double Nz() const noexcept { return nz; }
	double Tx() const noexcept { return tx; }
	double Ty() const noexcept { return ty; }
	double Tz() const noexcept { return tz; }
	double Sx() const noexcept { return sx; }
	double Sy() const noexcept { return sy; }
	double Sz() const noexcept { return sz; }

protected:
	double nx, ny, nz;
	double tx, ty, tz;
	double sx, sy, sz;

	void QuaternionToBase() noexcept {
		nx = 1 - 2*q2*q2 - 2*q3*q3;
		ny = 2*q1*q2 + 2*q3*q0;
		nz = 2*q1*q3 - 2*q2*q0;

		tx = 2*q1*q2 - 2*q3*q0;
		ty = 1 - 2*q1*q1 - 2*q3*q3;
		tz = 2*q2*q3 + 2*q1*q0;

		sx = 2*q1*q3 + 2*q2*q0;
		sy = 2*q2*q3 - 2*q1*q0;
		sz = 1 - 2*q1*q1 - 2*q2*q2;
	}

};
