#include "../../Includes/Contact/ContactDetectorSphereVersusDisk.h"

#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/PlanR.h"
#include "../../Includes/Contact/ContactDetectorBodyVersusDisk.h"

void ContactDetectorSphereVersusDisk::Detect(PlanR & p, Sphere *b, Contact *ct, int & Nct) noexcept {
	if(b->Bodies() != -9){
		ContactDetectorBodyVersusDisk::Detect(p,b->GetBody(),ct,Nct);
		return;
	}

	double px = b->X() - b->Radius()*p.Nx();
	double py = b->Y() - b->Radius()*p.Ny();
	double pz = b->Z() - b->Radius()*p.Nz();
	double pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
	double pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
	double ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
	if((pn >= -b->Radius()) && (pn < 0.0) && (sqrt(pt*pt+ps*ps) < p.Radius())){
		ct[Nct].delta = pn;
		ct[Nct].type = Contact::Type::SpherePlanR;// 2;
		ct[Nct].px = px;
		ct[Nct].py = py;
		ct[Nct].pz = pz;
		ct[Nct].nx = -p.Nx();
		ct[Nct].ny = -p.Ny();
		ct[Nct].nz = -p.Nz();
		ct[Nct].sa = b;
		ct[Nct].par = &p;
		Nct++;
	}
}
