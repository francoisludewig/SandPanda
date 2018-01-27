#include "../../Includes/Contact/ContactDetectorSphereVersusElbow.h"

#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Elbow.h"
#include "../../Includes/Contact/ContactDetectorBodyVersusElbow.h"

void ContactDetectorSphereVersusElbow::Detect(Elbow & p, Sphere *b, Contact *ct, int & Nct) noexcept {
	if(b->Bodies() != -9){
		ContactDetectorBodyVersusElbow::Detect(p,b->GetBody(),ct,Nct);
		return;
	}
	double alphal;
	double pn = (b->X()-p.x)*p.nx + (b->Y()-p.y)*p.ny + (b->Z()-p.z)*p.nz;
	double pt = (b->X()-p.x)*p.tx + (b->Y()-p.y)*p.ty + (b->Z()-p.z)*p.tz;
	double ps = (b->X()-p.x)*p.sx + (b->Y()-p.y)*p.sy + (b->Z()-p.z)*p.sz;
	double rl = sqrt(pn*pn+pt*pt);

	if(rl >= p.R-p.r && rl <= p.R+p.r && fabs(ps) <= p.r){
		// Coupe entre 2 plans "epaisseur"
		if(pt != 0){
			if(pn > 0)
				alphal = acos(pt/rl);
			else
				alphal = 2*M_PI-acos(pt/rl);
		}
		else{
			alphal = 0;
		}

		// control angulaire
		if(alphal <= p.alpha){
			double cx = p.x + (pn*p.nx + pt*p.tx)/rl*p.R;
			double cy = p.y + (pn*p.ny + pt*p.ty)/rl*p.R;
			double cz = p.z + (pn*p.nz + pt*p.tz)/rl*p.R;
			double D = sqrt(pow(b->X()-cx,2)+pow(b->Y()-cy,2)+pow(b->Z()-cz,2));
			if(p.r-(D+b->Radius()) < 0){
				ct[Nct].delta =  p.r-(D+b->Radius());
				ct[Nct].type = Contact::Type::SphereElbow;// 4;
				double nx = -(cx - b->X());
				double ny = -(cy - b->Y());
				double nz = -(cz - b->Z());
				double D = sqrt(nx*nx+ny*ny+nz*nz);
				ct[Nct].nx = nx/D;
				ct[Nct].ny = ny/D;
				ct[Nct].nz = nz/D;
				ct[Nct].px = b->X() + b->Radius()*ct[Nct].nx;
				ct[Nct].py = b->Y() + b->Radius()*ct[Nct].ny;
				ct[Nct].pz = b->Z() + b->Radius()*ct[Nct].nz;
				ct[Nct].sa = b;
				ct[Nct].ew = &p;
				Nct++;
			}
		}
	}
}
