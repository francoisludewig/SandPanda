#pragma once

#include <vector>
#include <cstdio>

class Cells {
public:
	Cells() = default;
	explicit Cells(const std::vector<int>& cells): cells(cells) {}
	explicit Cells(std::vector<int>&& cells): cells(std::move(cells)) {}
	~Cells() = default;

	Cells(const Cells& other) = default;
	Cells(Cells&& other) = default;
	Cells& operator=(const Cells& other) = default;
	Cells& operator=(Cells&& other) = default;

	[[nodiscard]] std::size_t Size() const { return cells.size(); }
	int& operator[](const std::size_t index) { return cells[index]; }
	const int& operator[](const std::size_t index) const { return cells[index]; }

private:
	std::vector<int> cells;
};
