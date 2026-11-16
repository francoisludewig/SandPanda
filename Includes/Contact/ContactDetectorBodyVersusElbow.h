#pragma once

class Elbow;
class Body;
class Contact;

class ContactDetectorBodyVersusElbow {
public:
    static void Detect(Elbow & p, Body *b, Contact *ct, int & Nct) noexcept;
};
