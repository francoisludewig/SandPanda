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
	/* Bloc de detection de contacts ; l'ordre des blocs est celui de la version sequentielle */
	struct DetectionTask {
		enum class Kind { SpherePlaneX, Plans, Disk, ConeRange, Elbow, HollowBalls };
		Kind kind;
		int index;   // plan X, disque, cone ou coude
		int begin;   // tranche de cellules (cones)
		int end;
	};
	/* Emplacement des contacts produits par un bloc : tampon du thread, debut et nombre */
	struct TaskOutput {
		int buffer{0};
		int start{0};
		int count{0};
		// Contributions compactes (forces, elongations) de ce bloc, par thread proprietaire
		int contributionBuffer{0};
		std::vector<int> forceStart, forceCount, xsiStart, xsiCount;
		void Resize(const int nOwners) {
			forceStart.assign(nOwners, 0);
			forceCount.assign(nOwners, 0);
			xsiStart.assign(nOwners, 0);
			xsiCount.assign(nOwners, 0);
		}
	};
	static constexpr int coneCellsPerTask = 256;

	void BuildDetectionTasks() noexcept;
	void RunDetectionTask(const DetectionTask& task, Contact *ct, int & Nct, const std::vector<Sphere*>& cell) noexcept;

	std::vector<DetectionTask> tasks;
	Contact *ct;
	SolidCells solidCells;
	CellBounds cellBounds;
    int Nct;
    bool isMultiThreads;
    std::shared_ptr<SimulationData> solids;
};
