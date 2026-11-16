#pragma once

class Body;
class Contact;

class ContactDetectorBodyVersusBody {
public:
    static int Detect(Body *a, Body *b, Contact *ct, int &Nct, double ra, double rb) noexcept;
};
