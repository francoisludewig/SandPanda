#pragma once

class Sphere;
class Body;
class Contact;

class ContactDetectorSphereVersusBody {
public:
	static int Detect(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept;
};
