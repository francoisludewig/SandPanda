#pragma once

class SolidCells {
	SolidCells() {}
	~SolidCells() {}
	SolidCells(const SolidCells& other) noexcept = default;
	SolidCells(SolidCells&& other) noexcept = default;
	SolidCells& operator=(const SolidCells& other) noexcept = default;
	SolidCells& operator=(SolidCells&& other) noexcept = default;

	void AddPlanCells(const Cells& cells) { planCells.push_back(cells); }
	void AddPlanCells(Cells&& cells) { planCells.push_back(cells); }
	void AddPlanRCells(const Cells& cells) { planRCells.push_back(cells); }
	void AddPlanRCells(Cells&& cells) { planRCells.push_back(cells); }
	void AddConeCells(const Cells& cells) { coneCells.push_back(cells); }
	void AddConeCells(Cells&& cells) { coneCells.push_back(cells); }
	void AddElbowCells(const Cells& cells) { elbowCells.push_back(cells); }
	void AddElbowCells(Cells&& cells) { elbowCells.push_back(cells); }

	const Cells& PlanCells(std::size_t index) const { return planCells[index]; }
	const Cells& PlanRCells(std::size_t index) const { return planRCells[index]; }
	const Cells& ConeCells(std::size_t index) const { return coneCells[index]; }
	const Cells& ElbowCells(std::size_t index) const { return elbowCells[index]; }

private:
	std::vector<Cells> planCells;
	std::vector<Cells> planRCells;
	std::vector<Cells> coneCells;
	std::vector<Cells> elbowCells;
};
