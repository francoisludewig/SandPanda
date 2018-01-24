#pragma once

#include <vector>
#include <cstdio>

class Cells {
public:
	Cells() {}
	Cells(const std::vector<int>& cells): cells(cells) {}
	Cells(std::vector<int>&& cells): cells(std::move(cells)) {}
	~Cells() {}

	Cells(const Cells& other) noexcept = default;
	Cells(Cells&& other) noexcept = default;
	Cells& operator=(const Cells& other) noexcept = default;
	Cells& operator=(Cells&& other) noexcept = default;

	std::size_t Size() { return cells.size(); }
	int& operator[](std::size_t index) { return cells[index]; }

private:
	std::vector<int> cells;
};
