#include "../../Includes/Object/PlanR.h"

PlanR::PlanR() noexcept : Solid() {
	r = 0.5;
	dn = 0;
	periodic = -9;
}

void PlanR::readFromFile(FILE *ft) noexcept {
	Solid::LoadFromFile(ft);	
	fscanf(ft,"%lf\t%lf\n",&dn,&r);
	fscanf(ft,"%d\n",&periodic);
	Solid::LoadAccelerationFromFile(ft);	
	Is = 1./4.*Mass*r*r;
	It = 1./4.*Mass*r*r;
	In = 1./2.*Mass*r*r;
}

void PlanR::writeToFile(FILE *ft) const  noexcept {
	Solid::WriteToFile(ft);	
	fprintf(ft,"%e\t%e\n",dn,r);
	fprintf(ft,"%d\n",periodic);
	Solid::WriteAccelerationFromFile(ft);
}

void PlanR::writeOutFile(FILE *ft, int mode) const noexcept {
	Solid::WriteOutFile(ft,mode);
    int Ngb = 0;
	if(mode == 0){
		fprintf(ft,"%e\t%e\n",dn,r);
		fprintf(ft,"%d\t%d\n",periodic, Ngb);
	}
	else{
		fwrite(&dn, sizeof(double), 1, ft);
		fwrite(&r, sizeof(double), 1, ft);
		fwrite(&periodic, sizeof(int), 1, ft);
		fwrite(&Ngb, sizeof(int), 1, ft);
	}
}

