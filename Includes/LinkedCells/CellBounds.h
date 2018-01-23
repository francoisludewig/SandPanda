#pragma once

class CellBounds {
public:
	CellBounds() noexcept :
		minX(0), minY(0), minZ(0),
		maxX(0), maxY(0), maxZ(0) {}

	CellBounds(int minX, int minY, int minZ,
			   int maxX, int maxY, int maxZ) noexcept :
			minX(minX), minY(minY), minZ(minZ),
			maxX(maxX), maxY(maxY), maxZ(maxZ) {}

	CellBounds(const CellBounds& other) noexcept = default;
	CellBounds(CellBounds&& other) noexcept = default;
	CellBounds& operator=(const CellBounds& other) noexcept = default;
	CellBounds& operator=(CellBounds&& other) noexcept = default;

	~CellBounds() {}

	int MinX() const noexcept { return minX;}
	int MinY() const noexcept { return minY;}
	int MinZ() const noexcept { return minZ;}
	int MaxX() const noexcept { return maxX;}
	int MaxY() const noexcept { return maxY;}
	int MaxZ() const noexcept { return maxZ;}

private:
	int minX, minY, minZ;
	int maxX, maxY, maxZ;
};
