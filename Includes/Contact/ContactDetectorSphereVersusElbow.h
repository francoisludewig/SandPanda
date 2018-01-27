#pragma once

class Elbow;
class Sphere;
class Contact;

class ContactDetectorSphereVersusElbow {
public:
	static void Detect(Elbow & p, Sphere *b, Contact *ct, int & Nct) noexcept;
};
