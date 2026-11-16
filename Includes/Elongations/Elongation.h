#pragma once
#include <iostream>

class Elongation{
public:
	Elongation() noexcept;
	
	Elongation(const Elongation& other) noexcept = default;
	Elongation(Elongation&& other) noexcept = default;
	Elongation& operator=(const Elongation& other) noexcept = default;
	Elongation& operator=(Elongation&& other) noexcept = default;

	static Elongation Empty() noexcept {
		static Elongation elongation;
		return elongation;
	}

	void Reset() noexcept;
	void Display() const noexcept;
	void Rotate(double nx, double ny, double nz) noexcept;
	// Elongation vue depuis l'autre corps du contact : -xsi, meme statut
	[[nodiscard]] Elongation Opposite() const noexcept {
		Elongation e(*this);
		e.x = -x;
		e.y = -y;
		e.z = -z;
		return e;
	}
	
	double x,y,z;
	int status;
};
