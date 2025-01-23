#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include "Sphere.h"
#include "MasterSolid.h"

class Data{
public:
	Data() noexcept;
	
	Data(const Data& other) noexcept = default;
	Data(Data&& other) noexcept = default;
	Data& operator=(const Data& other) noexcept = default;
	Data& operator=(Data&& other) noexcept = default;
	
	~Data() noexcept;
	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft) const noexcept;
	double DilatationCoefficient() const noexcept;
	void LoadFluid(FILE *ft) noexcept;
	void ComputeRmax(std::vector<Sphere> & sph, int Nsph) noexcept;
	void ComputeLinkedCell() noexcept;
	
	double dt,TIME,Total;
	int Nsp;
	double xmin,ymin,zmin;
	double xmax,ymax,zmax;
	double ax,ay,az;
	int Nx,Ny,Nz;
	double en,mu,k,muS,muD;
	double dts,t0;
	int modelTg;
	double Rmax;
	double Rthreshold;
	int Ncellmax;
	
	MasterSolid *mas;
	// Donnee relative aux options de la simulation
	int outContact;
	int outMode;
};
