#pragma once

#include "Cells.h"

class SolidCells {
public:
	SolidCells() = default;
	~SolidCells() = default;
	SolidCells(const SolidCells& other) = default;
	SolidCells(SolidCells&& other) = default;
	SolidCells& operator=(const SolidCells& other) = default;
	SolidCells& operator=(SolidCells&& other) = default;

	void AddPlanCells(Cells&& cells) { planCells.push_back(std::move(cells)); }
	void AddPlanRCells(Cells&& cells) { planRCells.push_back(std::move(cells)); }
	void AddConeCells(Cells&& cells) { coneCells.push_back(std::move(cells)); }
	void AddElbowCells(Cells&& cells) { elbowCells.push_back(std::move(cells)); }

	[[nodiscard]] const Cells& PlanCells(const std::size_t index) const { return planCells[index]; }
	[[nodiscard]] const Cells& PlanRCells(const std::size_t index) const { return planRCells[index]; }
	[[nodiscard]] const Cells& ConeCells(const std::size_t index) const { return coneCells[index]; }
	[[nodiscard]] const Cells& ElbowCells(const std::size_t index) const { return elbowCells[index]; }

private:
	std::vector<Cells> planCells;
	std::vector<Cells> planRCells;
	std::vector<Cells> coneCells;
	std::vector<Cells> elbowCells;
};
