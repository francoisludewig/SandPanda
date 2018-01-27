#pragma once

class PlanR;
class Body;
class Contact;

class ContactDetectorBodyVersusDisk {
public:
	static void Detect(PlanR & p, Body *b, Contact *ct, int & Nct) noexcept;
};
