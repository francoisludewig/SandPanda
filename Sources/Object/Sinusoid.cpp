#include "../../Includes/Object/Sinusoid.h"

#include <cmath>

Sinusoid::Sinusoid() noexcept :
a0(0), a1(0), w(0), phi(0)  {}

Sinusoid::Sinusoid(double a0) noexcept :
a0(a0), a1(0), w(0), phi(0)  {}

Sinusoid::Sinusoid(double a0, double a1, double w, double phi) noexcept :
a0(a0), a1(a1), w(w), phi(phi) {}

void Sinusoid::LoadFromFile(FILE *ft) noexcept {
  if(fscanf(ft,"%lf\t%lf\t%lf\t%lf\n", &a0, &a1, &w, &phi) != 4) {
      printf("Error when loading Sinusoid !\n");
  }
}

void Sinusoid::WriteToFile(FILE *ft, int mode) const noexcept {
	if(mode == 0){
		fprintf(ft,"%e\t%e\t%e\t%e\n",a0,a1,w,phi);
	}
	else{
		fwrite(&a0, sizeof(double), 1, ft);
		fwrite(&a1, sizeof(double), 1, ft);
		fwrite(&w, sizeof(double), 1, ft);
		fwrite(&phi, sizeof(double), 1, ft);
	}
}

void Sinusoid::Display() const noexcept {
	printf("%e\t%e\t%e\t%e\n",a0,a1,w,phi);
}

double Sinusoid::Value(double t) const noexcept {
  return (a0+a1*sin(w*t+phi));
}

double Sinusoid::Max() const noexcept {
	 return (fabs(a0)+fabs(a1));
}


/*
 *  Return the duration of the movement if the Sinusoid represent a translational velocity
 *  In case of oscillation, return the period
 *  In case of constant and non null velocity => 10000 as infinity value
 *  In case of constant and null velocity => 0
 */
double Sinusoid::DelayTr() const noexcept {
	if(w != 0)
		return((2.*M_PI)/fabs(w));
	else{
		if(a0 != 0)
			return(10000.0);
		else
			return(0.0);
	}
}

/*
 *  Return the duration of the movement if the Sinusoid represent a rotationnal velocity
 *  In case of oscillation, return the period
 *  In case of constant and non null velocity => 10000 as infinity value
 *  In case of constant and null velocity => 0
 */
double Sinusoid::DelayRot() const noexcept {
	double Wmax = fabs(a0)+fabs(a1);
	if(Wmax != 0)
		return((2.*M_PI)/Wmax);
	else{
		return(0.0);
	}
}
