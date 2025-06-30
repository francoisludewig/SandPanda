#pragma once

#include <vector>
#include "../Solids/Sphere.h"
#include "../Solids/MasterSolid.h"

class Configuration{
public:
	Configuration() noexcept;
	
	Configuration(const Configuration& other) noexcept = default;
	Configuration(Configuration&& other) noexcept = default;
	Configuration& operator=(const Configuration& other) noexcept = default;
	Configuration& operator=(Configuration&& other) noexcept = default;
	
	~Configuration() noexcept;
	void LoadFromFile(FILE *ft) noexcept;
	void WriteToFile(FILE *ft) const noexcept;
	void ComputeRmax(std::vector<Sphere> & sph) noexcept;
	void ComputeLinkedCell() noexcept;
	
	double dt,TIME,Total;
	int Nsp;
	double xmin,ymin,zmin;
	double xmax,ymax,zmax;
	double ax,ay,az;
	int Nx,Ny,Nz;
	double en,mu,k,muS,muD;
	double dts,t0;
	int modelTg{};
	double Rmax{};
	double Rthreshold{};
	int Ncellmax{};
	
	MasterSolid *mas{};
	// Donnee relative aux options de la simulation
	int outContact;
	int outMode;

	bool record{};
};
