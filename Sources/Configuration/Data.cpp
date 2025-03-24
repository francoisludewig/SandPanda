#include "../../Includes/Configuration/Data.h"

Data::Data() noexcept :
	dt(0), TIME(0), Total(0), Nsp(0), xmin(0), ymin(0), zmin(0),
	xmax(0), ymax(0), zmax(0), ax(0), ay(0), az(0), Nx(0), Ny(0), Nz(0),
  en(0), mu(0), k(0), muS(0), muD(0), dts(0), t0(0), outContact(0), outMode(0) {}

Data::~Data() noexcept = default;

void Data::LoadFromFile(FILE *ft) noexcept{
	fscanf(ft,"%lf\t%lf\t%lf\n",&dt,&TIME,&Total);
	fscanf(ft,"%d\n",&Nsp);
    
    fscanf(ft,"%d\t",&modelTg);

    if(modelTg == 0)
		fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&en,&mu,&mu,&k);
	else
		fscanf(ft,"%lf\t%lf\t%lf\t%lf\n",&en,&muS,&muD,&k);			
	fscanf(ft,"%lf\t%lf\n",&dts,&t0);
	fscanf(ft,"%lf\t%lf\t%lf\n",&xmin,&ymin,&zmin);
	fscanf(ft,"%lf\t%lf\t%lf\n",&xmax,&ymax,&zmax);
	fscanf(ft,"%lf\t%lf\t%lf\n",&ax,&ay,&az);
	fscanf(ft,"%d\t%d\t%d\n",&Nx,&Ny,&Nz);

    // Affichage

    printf("\nNomral : e = %e, kn = %e\n", en, k);
    printf("\nModel Tg : %s\n",modelTg==0 ? "Dynamic" : "Static");
    if(modelTg == 0)
        printf("mu = %e\n",&mu);
    else
        printf("mus = %e, mud = %e\n",muS,muD);

    printf("\n%e < x < %e\n",xmin,xmax);
	printf("%e < y < %e\n",ymin,ymax);
	printf("%e < z < %e\n",zmin,zmax);
	printf("N = (%d,%d,%d)\n",Nx,Ny,Nz);
	printf("a = (%e,%e,%e)\n\n",ax,ay,az);
}

void Data::WriteToFile(FILE *ft) const noexcept{
	fprintf(ft,"%e\t%e\t%e\n",dt,TIME,Total);
	fprintf(ft,"%d\n",Nsp);
    if(modelTg == 0)
		fprintf(ft,"%d\t%e\t%e\t%e\t%e\n",modelTg,en,mu,mu,k);
	else
		fprintf(ft,"%d\t%e\t%e\t%e\t%e\n",modelTg,en,muS,muD,k);
	fprintf(ft,"%e\t%e\n",dts,t0);
	fprintf(ft,"%e\t%e\t%e\n",xmin,ymin,zmin);
	fprintf(ft,"%e\t%e\t%e\n",xmax,ymax,zmax);
	fprintf(ft,"%e\t%e\t%e\n",ax,ay,az);
	fprintf(ft,"%d\t%d\t%d\n",Nx,Ny,Nz);
}

void Data::ComputeRmax(std::vector<Sphere> & sph, int Nsph) noexcept{
	Rmax = sph[0].Radius();
	for(int i = 1 ; i < Nsph ; i++){
		if(Rmax < sph[i].Radius())Rmax = sph[i].Radius();
	}
}

void Data::ComputeLinkedCell() noexcept{
	ax = Rmax*2.01;
	ay = Rmax*2.01;
	az = Rmax*2.01;
	Nx = (int)((xmax-xmin)/ax);
	Ny = (int)((ymax-ymin)/ay);
	Nz = (int)((zmax-zmin)/az);
	ax = (xmax-xmin)/Nx;
	ay = (ymax-ymin)/Ny;
	az = (zmax-zmin)/Nz;
	printf("\nR* = %e\n",Rmax);
	printf("N = (%d,%d,%d)\n",Nx,Ny,Nz);
	printf("a = (%e,%e,%e)\n\n",ax,ay,az);
}