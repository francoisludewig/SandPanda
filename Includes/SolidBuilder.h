#pragma once

#include <memory>

class Option;
class Solids;

class SolidBuilder {
public:
	static std::shared_ptr<Solids> BuildFromExport(Option &opt) noexcept;
	static std::shared_ptr<Solids> BuildFromStart_Stop(Option& opt) noexcept;
	static void WriteStart_Stop(Option &opt, std::shared_ptr<Solids>& solids, int ntp) noexcept;
private:
	static void LoadBodySpecies(Option &opt, std::shared_ptr<Solids>& solids) noexcept;
};
