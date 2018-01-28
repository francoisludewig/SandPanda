#pragma once

class CellBounds {
public:
	CellBounds() noexcept :
		minX(0), minY(0), minZ(0),
		maxX(0), maxY(0), maxZ(0),
		startX(0), startY(0), startZ(0),
		endX(0), endY(0), endZ(0),
		lx(0), ly(0), lz(0){}

	CellBounds(int minX, int minY, int minZ,
			   int maxX, int maxY, int maxZ,
			   int startX, int startY, int startZ,
			   int endX, int endY, int endZ,
			   double lx, double ly, double lz) noexcept :
			minX(minX), minY(minY), minZ(minZ),
			maxX(maxX), maxY(maxY), maxZ(maxZ),
			startX(startX), startY(startY), startZ(startZ),
			endX(endX), endY(endY), endZ(endZ),
			lx(lx), ly(ly), lz(lz){}


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
	int StartX() const noexcept { return startX;}
	int StartY() const noexcept { return startY;}
	int StartZ() const noexcept { return startZ;}
	int EndX() const noexcept { return endX;}
	int EndY() const noexcept { return endY;}
	int EndZ() const noexcept { return endZ;}
	double Lx() const noexcept { return lx;}
	double Ly() const noexcept { return ly;}
	double Lz() const noexcept { return lz;}

private:
	int minX, minY, minZ;
	int maxX, maxY, maxZ;
	int startX, startY, startZ;
	int endX, endY, endZ;
	double lx, ly, lz;
};
