#pragma once

class Plan;
class Body;
class Contact;

class ContactDetectorBodyVersusPlan {
public:
	static void Detect(Plan & p, Body *b, Contact *ct, int & Nct) noexcept;
};
