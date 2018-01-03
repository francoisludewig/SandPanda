#include "../Includes/Plan.h"
#include "../Includes/Contact.h"
#include "../Includes/Sphere.h"

Plan::Plan() noexcept : Solid(){
	dt = 0.5;
	ds = 0.5;
	dn = 0;
	periodic = -9;
	sigma = 0;
	ra = 0;
}

Plan::~Plan() noexcept {
	delete [] list;
}

void Plan::InitList(int N) noexcept {
	if(periodic != -9)
		list = new int[N];
}

double Plan::Ds() const noexcept {
	return ds;
}

double Plan::Dt() const noexcept {
	return dt;
}

void Plan::SetAlpha(double a) noexcept {
	sigma = cos(a);
	printf("Sigma = %e\n",sigma);
}

void Plan::Normal(Contact *c, Sphere *s) noexcept {
	double r,phi,theta;
	if(sigma != 0){
		if(s->Ct_pl() == 0){
			r = sigma + ((double)(rand()%RAND_MAX)/RAND_MAX)*(1.-sigma);
			phi = acos(r);
			theta = ((double)(rand()%RAND_MAX)/RAND_MAX)*2*M_PI;
			c->nx = sin(phi)*cos(theta);
			c->ny = sin(phi)*sin(theta);
			c->nz = -cos(phi);
			s->Ct_pl_nx(c->nx);
			s->Ct_pl_ny(c->ny);
			s->Ct_pl_nz(c->nz);
			s->Ct_pl (1);
		}
		else{
			c->nx = s->Ct_pl_nx();
			c->ny = s->Ct_pl_ny();
			c->nz = s->Ct_pl_nz();
			s->Ct_pl(1);
		}
	}
}

void Plan::LoadFromFile(FILE *ft) noexcept {
	Solid::LoadFromFile(ft);
	fscanf(ft,"%lf\t%lf\t%lf\n",&dn,&dt,&ds);
	fscanf(ft,"%d\t%d\n",&periodic,&inAndOut);
	Solid::LoadAccelerationFromFile(ft);
	Is = 1./3.*Mass*ds*ds;
	It = 1./3.*Mass*dt*dt;
	In = 1./3.*Mass*(dt*dt+ds*ds);
}

void Plan::Display() const noexcept {
    printf("Num : %d\n",numero);
    printf("p = (%e,%e,%e)\n",x,y,z);
    printf("q = (%e,%e,%e,%e)\n",q0,q1,q2,q3);
    printf("n = (%e,%e,%e)\n\n",nx,ny,nz);
}

void Plan::WriteToFile(FILE *ft) const noexcept {
	Solid::WriteToFile(ft);
	fprintf(ft,"%e\t%e\t%e\n",dn,dt,ds);
	fprintf(ft,"%d\t%d\n",periodic,inAndOut);
	Solid::WriteAccelerationFromFile(ft);
}

void Plan::WriteOutFile(FILE *ft, int mode) const noexcept {
	Solid::WriteOutFile(ft,mode);
	if(mode == 0){
		fprintf(ft,"%e\t%e\t%e\n",0.0,dt,ds);
		fprintf(ft,"%d\t%d\t%d\n",periodic,Ngb,inAndOut);
	}
	else{
		double a = 0.0;
		fwrite(&a, sizeof(double), 1, ft);
		fwrite(&dt, sizeof(double), 1, ft);
		fwrite(&ds, sizeof(double), 1, ft);
		fwrite(&periodic, sizeof(int), 1, ft);
		fwrite(&Ngb, sizeof(int), 1, ft);
		fwrite(&inAndOut, sizeof(int), 1, ft);
	}
}

