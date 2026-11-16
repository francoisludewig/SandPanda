#pragma once

class PlanR;
class Sphere;
class Contact;

class ContactDetectorSphereVersusDisk {
public:
	static void Detect(PlanR & p, Sphere *b, Contact *ct, int & Nct) noexcept;
};
