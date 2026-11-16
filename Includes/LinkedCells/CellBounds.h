#pragma once

class CellBounds {
public:
	CellBounds() noexcept :
		minX(0), minY(0), minZ(0),
		maxX(0), maxY(0), maxZ(0),
		startX(0), startY(0), startZ(0),
		endX(0), endY(0), endZ(0),
		lx(0), ly(0), lz(0),
		xMin(0), yMin(0), zMin(0){}

	CellBounds(int minX, int minY, int minZ,
			   int maxX, int maxY, int maxZ,
			   int startX, int startY, int startZ,
			   int endX, int endY, int endZ,
			   double lx, double ly, double lz,
			   double xMin, double yMin, double zMin) noexcept :
			minX(minX), minY(minY), minZ(minZ),
			maxX(maxX), maxY(maxY), maxZ(maxZ),
			startX(startX), startY(startY), startZ(startZ),
			endX(endX), endY(endY), endZ(endZ),
			lx(lx), ly(ly), lz(lz),
			xMin(xMin), yMin(yMin), zMin(zMin){}


	CellBounds(const CellBounds& other) noexcept = default;
	CellBounds(CellBounds&& other) noexcept = default;
	CellBounds& operator=(const CellBounds& other) noexcept = default;
	CellBounds& operator=(CellBounds&& other) noexcept = default;

	~CellBounds() = default;

	[[nodiscard]]int MinX() const noexcept { return minX;}
	[[nodiscard]]int MinY() const noexcept { return minY;}
	[[nodiscard]]int MinZ() const noexcept { return minZ;}
	[[nodiscard]]int MaxX() const noexcept { return maxX;}
	[[nodiscard]]int MaxY() const noexcept { return maxY;}
	[[nodiscard]]int MaxZ() const noexcept { return maxZ;}
	[[nodiscard]]int StartX() const noexcept { return startX;}
	[[nodiscard]]int StartY() const noexcept { return startY;}
	[[nodiscard]]int StartZ() const noexcept { return startZ;}
	[[nodiscard]]int EndX() const noexcept { return endX;}
	[[nodiscard]]int EndY() const noexcept { return endY;}
	[[nodiscard]]int EndZ() const noexcept { return endZ;}
	[[nodiscard]]double Lx() const noexcept { return lx;}
	[[nodiscard]]double Ly() const noexcept { return ly;}
	[[nodiscard]]double Lz() const noexcept { return lz;}
	[[nodiscard]]double XMin() const noexcept { return xMin;}
	[[nodiscard]]double YMin() const noexcept { return yMin;}
	[[nodiscard]]double ZMin() const noexcept { return zMin;}

	[[nodiscard]] int CellCount() const noexcept {return (endX-startX)*(endY-startY)*(endZ-startZ);}
	[[nodiscard]] int Index(const int xIndex, const int yIndex, const int zIndex) const noexcept { return (xIndex*maxY*maxZ+yIndex*maxZ+zIndex); }

private:
	int minX, minY, minZ;
	int maxX, maxY, maxZ;
	int startX, startY, startZ;
	int endX, endY, endZ;
	double lx, ly, lz;
	double xMin, yMin, zMin;
};
