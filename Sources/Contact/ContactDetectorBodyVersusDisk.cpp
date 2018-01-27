#include "../../Includes/Contact/ContactDetectorBodyVersusDisk.h"

#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Solids/PlanR.h"

void ContactDetectorBodyVersusDisk::Detect(PlanR & p, Body *b, Contact *ct, int & Nct) noexcept {
	double px,py,pz,pn,pt,ps;
	for(int i = 0 ; i < b->SphereCount() ; i++){
		px = b->SphereX(i) - b->SphereRadius(i)*p.Nx();
		py = b->SphereY(i) - b->SphereRadius(i)*p.Ny();
		pz = b->SphereZ(i) - b->SphereRadius(i)*p.Nz();
		pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
		pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
		ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
		if((pn >= -b->SphereRadius(i)) && (pn < 0.0) && (sqrt(pt*pt+ps*ps) < p.Radius())){
			ct[Nct].delta = pn;
			ct[Nct].type = Contact::Type::BodyPlanR;// 12;
			ct[Nct].px = px;
			ct[Nct].py = py;
			ct[Nct].pz = pz;
			ct[Nct].nx = -p.Nx();
			ct[Nct].ny = -p.Ny();
			ct[Nct].nz = -p.Nz();
			ct[Nct].ba = b;
			ct[Nct].nba = i;
			ct[Nct].par = &p;
			Nct++;
		}
	}
}
