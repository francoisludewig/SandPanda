#pragma once

class Cone;
class Body;
class Contact;

class ContactDetectorBodyVersusCone {
public:
    static void Detect(Cone & p, Body *b, Contact *ct, int & Nct) noexcept;
};
