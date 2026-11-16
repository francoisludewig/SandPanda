#pragma once

class Sphere;
class Contact;

class ContactDetectorSphereVersusSphere {
public:
	static int Detect(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept;
};
