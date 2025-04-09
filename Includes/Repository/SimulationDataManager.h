#pragma once

#include <memory>

class Option;
class SimulationData;

class SimulationDataManager {
public:
	static std::shared_ptr<SimulationData> FromExport(Option &opt) noexcept;
	static std::shared_ptr<SimulationData> FromStart_Stop(Option& opt) noexcept;
	static void WriteStart_Stop(Option &opt, std::shared_ptr<SimulationData>& solids, int ntp) noexcept;
private:
	static void LoadBodySpecies(Option &opt, std::shared_ptr<SimulationData>& solids) noexcept;
};
