#include "../../Includes/Configuration/Configuration.h"

Configuration::Configuration() noexcept :
	dt(0), TIME(0), Total(0), Nsp(0), x_min(0), y_min(0), z_min(0),
	x_max(0), y_max(0), z_max(0), ax(0), ay(0), az(0), Nx(0), Ny(0), Nz(0),
  en(0), mu(0), k(0), muS(0), muD(0), dts(0), t0(0), outContact(0), outMode(0) {}

Configuration::~Configuration() noexcept = default;

void Configuration::LoadFromFile(FILE *ft) noexcept{
	fscanf(ft,"%lf\t%lf\t%lf\n",&dt,&TIME,&Total);
	fscanf(ft,"%d\n",&Nsp);
    
    fscanf(ft,"%d\t",&modelTg);

    if(modelTg == 0)
		fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&en,&mu,&mu,&k);
	else
		fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&en,&muS,&muD,&k);			
	fscanf(ft,"%lf\t%lf\n",&dts,&t0);
	fscanf(ft,"%lf\t%lf\t%lf\n",&x_min,&y_min,&z_min);
	fscanf(ft,"%lf\t%lf\t%lf\n",&x_max,&y_max,&z_max);
	fscanf(ft,"%lf\t%lf\t%lf\n",&ax,&ay,&az);
	fscanf(ft,"%d\t%d\t%d\n",&Nx,&Ny,&Nz);

    // Affichage
	printf("\n%e < x < %e\n",x_min,x_max);
	printf("%e < y < %e\n",y_min,y_max);
	printf("%e < z < %e\n",z_min,z_max);
	printf("N = (%d,%d,%d)\n",Nx,Ny,Nz);
	printf("a = (%e,%e,%e)\n\n",ax,ay,az);
}

void Configuration::WriteToFile(FILE *ft) const noexcept{
	fprintf(ft,"%e\t%e\t%e\n",dt,TIME,Total);
	fprintf(ft,"%d\n",Nsp);
    if(modelTg == 0)
		fprintf(ft,"%d\t%e\t%e\t%e\t%e\n",modelTg,en,mu,mu,k);
	else
		fprintf(ft,"%d\t%e\t%e\t%e\t%e\n",modelTg,en,muS,muD,k);
	fprintf(ft,"%e\t%e\n",dts,t0);
	fprintf(ft,"%e\t%e\t%e\n",x_min,y_min,z_min);
	fprintf(ft,"%e\t%e\t%e\n",x_max,y_max,z_max);
	fprintf(ft,"%e\t%e\t%e\n",ax,ay,az);
	fprintf(ft,"%d\t%d\t%d\n",Nx,Ny,Nz);
}

void Configuration::ComputeRmax(std::vector<Sphere> & sph) noexcept{
	Rmax = sph[0].Radius();
	for(const auto& sphere : sph){
		if(Rmax < sphere.Radius())Rmax = sphere.Radius();
	}
}

void Configuration::ComputeLinkedCell() noexcept{
	ax = Rmax*2.01;
	ay = Rmax*2.01;
	az = Rmax*2.01;
	Nx = (int)((x_max-x_min)/ax);
	Ny = (int)((y_max-y_min)/ay);
	Nz = (int)((z_max-z_min)/az);
	ax = (x_max-x_min)/Nx;
	ay = (y_max-y_min)/Ny;
	az = (z_max-z_min)/Nz;
	printf("\nR* = %e\n",Rmax);
	printf("N = (%d,%d,%d)\n",Nx,Ny,Nz);
	printf("a = (%e,%e,%e)\n\n",ax,ay,az);
}
