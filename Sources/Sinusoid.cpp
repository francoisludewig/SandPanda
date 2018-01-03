#include "../Includes/Sinusoid.h"


Sinusoid::Sinusoid() noexcept :
a0(0), a1(0), w(0), phi(0)  {}

Sinusoid::Sinusoid(double a0, double a1, double w, double phi) noexcept :
a0(a0), a1(a1), w(w), phi(phi) {}

void Sinusoid::LoadFromFile(FILE *ft) noexcept {
  fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&a0,&a1,&w,&phi);
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

double Sinusoid::Max() const noexcept {
	 return (fabs(a0)+fabs(a1));
}

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

double Sinusoid::DelayRot() const noexcept {
	double Wmax = fabs(a0)+fabs(a1);
	if(Wmax != 0)
		return((2.*M_PI)/Wmax);
	else{
		return(0.0);
	}
}
