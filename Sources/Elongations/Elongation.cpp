#include "../../Includes/Elongations/Elongation.h"

Elongation::Elongation() noexcept : x(0), y(0), z(0), status(0) {}

void Elongation::Reset() noexcept {
	x = 0;
	y = 0;
	z = 0;
	status = 0;
}

void Elongation::Rotate(const double nx, const double ny, const double nz) noexcept {
	const double norm = x*x+y*y+z*z;
	double ps = x*nx+y*ny+z*nz;
	x -= ps*nx;
	y -= ps*ny;
	z -= ps*nz;
	ps = norm/(x*x+y*y+z*z);
	x *= ps;
	y *= ps;
	z *= ps;
}

void Elongation::Display() const noexcept {
	printf("xsi = (%e,%e,%e) status = %d\n",x,y,z,status);
}
