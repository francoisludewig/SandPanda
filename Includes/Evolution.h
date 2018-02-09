#pragma once

#include "Contact/Contact.h"

#include "LinkedCells/SolidCells.h"
#include "LinkedCells/Cells.h"
#include "LinkedCells/SolidCellsBuilder.h"
#include "LinkedCells/CellBounds.h"
#include "Solids/SimulationData.h"

#include <vector>
#include <memory>

class Sphere;
class Plan;
class PlanR;
class Cone;
class Elbow;
class Body;
class HollowBall;
class Configuration;
class Gravity;

class Evolution{
public:
	Evolution(std::shared_ptr<SimulationData>& solids,
			  const CellBounds& cellBounds,
			  bool isMultiThreads) :
		cellBounds(cellBounds),
		isMultiThreads(isMultiThreads),
		solids (solids){
		ct = new Contact[18*solids->spheres.size()+75*solids->bodies.size()];
		solidCells = SolidCellsBuilder::Build(solids->configuration, solids->plans, solids->disks, solids->cones, solids->elbows, solids->gravity, cellBounds);
		Nct = 0;
	}

	~Evolution() noexcept { delete[] ct;}

	Contact* GetContacts() noexcept { return ct; }
	int ContactCount() const noexcept { return Nct; }

    int Evolve(std::vector<Sphere*>& cell, int & Ntp, char *name, int Nthreshold) noexcept;
	
    int EvolveMelt(std::vector<Sphere*> cell, int & Ntp, char *name, double vr, double delayVr, int Nthreshold) noexcept;

private:
	Contact *ct;
	SolidCells solidCells;
	CellBounds cellBounds;
    int Nct;
    bool isMultiThreads;
    std::shared_ptr<SimulationData> solids;
};
