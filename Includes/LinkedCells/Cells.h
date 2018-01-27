#pragma once

#include <vector>
#include <cstdio>

class Cells {
public:
	Cells() {}
	Cells(const std::vector<int>& cells): cells(cells) {}
	Cells(std::vector<int>&& cells): cells(std::move(cells)) {}
	~Cells() {}

	Cells(const Cells& other) = default;
	Cells(Cells&& other) = default;
	Cells& operator=(const Cells& other) = default;
	Cells& operator=(Cells&& other) = default;

	std::size_t Size() const { return cells.size(); }
	int& operator[](std::size_t index) { return cells[index]; }
	const int& operator[](std::size_t index) const { return cells[index]; }

private:
	std::vector<int> cells;
};
