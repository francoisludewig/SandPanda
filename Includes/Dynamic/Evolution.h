#pragma once

#include "../Contact/Contact.h"

#include "../LinkedCells/SolidCells.h"
#include "../LinkedCells/Cells.h"
#include "../LinkedCells/SolidCellsBuilder.h"
#include "../LinkedCells/CellBounds.h"
#include "../Repository//SimulationData.h"

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
	Evolution(const std::shared_ptr<SimulationData>& solids,
			  const CellBounds& cellBounds,
			  const bool isMultiThreads) :
		cellBounds(cellBounds),
		isMultiThreads(isMultiThreads),
		solids (solids){
		ct = nullptr;
		solidCells = SolidCellsBuilder::Build(solids->configuration, solids->plans, solids->disks, solids->cones, solids->elbows, solids->gravity, cellBounds);
		Nct = 0;
	}

	~Evolution() noexcept = default;

	[[nodiscard]] Contact* GetContacts() const noexcept { return ct; }
	[[nodiscard]] int ContactCount() const noexcept { return Nct; }

    int Evolve(std::vector<Sphere*>& cell, int & Ntp, char *name, bool isMonitoringActivated) noexcept;

private:
	Contact *ct;
	SolidCells solidCells;
	CellBounds cellBounds;
    int Nct;
    bool isMultiThreads;
    std::shared_ptr<SimulationData> solids;
};
