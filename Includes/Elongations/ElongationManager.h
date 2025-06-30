#pragma once

#include "Elongation.h"

#include <memory>
#include <vector>

class ElongationManager {
private:
	// Data static contact
	std::vector<Elongation> xsi, tp_xsi;
	std::vector<uint64_t> id, tp_id;
	int count,tp_count;
public:
	explicit ElongationManager(int maxContact) noexcept;
	~ElongationManager() noexcept = default;
	void InitXsi() noexcept;
	bool AddXsi(Elongation& e, uint64_t contact_id) noexcept;
	[[nodiscard]] Elongation FoundIt(uint64_t contact_id) const noexcept;
	void writeToFile(FILE *ft) const noexcept;
	void readFromFile(FILE *ft) noexcept;
};
