#pragma once

class Cone;
class Sphere;
class Contact;

class ContactDetectorSphereVersusCone {
public:
	static void Detect(Cone & p, Sphere *b, Contact *ct, int & Nct) noexcept;
};
