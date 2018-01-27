#pragma once

class Plan;
class Sphere;
class Contact;

class ContactDetectorSphereVersusPlan {
public:
	static void Detect(Plan & p, Sphere *b, Contact *ct, int & Nct) noexcept;
};
