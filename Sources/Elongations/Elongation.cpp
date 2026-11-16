#include "../../Includes/Elongations/Elongation.h"

#include <cmath>

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
	// Remise a l'echelle pour conserver la norme de xsi apres projection
	// sur le nouveau plan tangent (rapport des normes, pas de leurs carres)
	if(const double norm2 = x*x+y*y+z*z; norm2 > 0.){
		ps = sqrt(norm/norm2);
		x *= ps;
		y *= ps;
		z *= ps;
	}
}

void Elongation::Display() const noexcept {
	printf("xsi = (%e,%e,%e) status = %d\n",x,y,z,status);
}
