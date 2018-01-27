#include "../../Includes/Contact/ContactDetectorBodyVersusPlan.h"

#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Solids/Plan.h"

void ContactDetectorBodyVersusPlan::Detect(Plan & p, Body *b, Contact *ct, int & Nct) noexcept {
	for(int i = 0 ; i < b->SphereCount() ; i++){
		double px = b->SphereX(i) - b->SphereRadius(i)*p.Nx();
		double py = b->SphereY(i) - b->SphereRadius(i)*p.Ny();
		double pz = b->SphereZ(i) - b->SphereRadius(i)*p.Nz();
		double pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
		double pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
		double ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
		if((fabs(pt) < p.Dt()) && (fabs(ps) < p.Ds())){
			if( (pn <= 0.0) && (pn > -b->SphereRadius(i))){
				ct[Nct].type = Contact::Type::BodyPlan;// 11;
				ct[Nct].delta = pn;
				ct[Nct].px = px;
				ct[Nct].py = py;
				ct[Nct].pz = pz;
				ct[Nct].nx = -p.Nx();
				ct[Nct].ny = -p.Ny();
				ct[Nct].nz = -p.Nz();
				ct[Nct].pa = &p;
				ct[Nct].ba = b;
				ct[Nct].nba = i;
				Nct++;
			}
			if(p.InAndOut() == 1){
				px = b->SphereX(i) + b->SphereRadius(i)*p.Nx();
				py = b->SphereY(i) + b->SphereRadius(i)*p.Ny();
				pz = b->SphereZ(i) + b->SphereRadius(i)*p.Nz();
				pn = (px-p.X())*(-p.Nx())+(py-p.Y())*(-p.Ny())+(pz-p.Z())*(-p.Nz());
				pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
				ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
				if( (pn <= 0.0) && (pn > -b->SphereRadius(i))){
					ct[Nct].type = Contact::Type::BodyPlan;// 11;
					ct[Nct].delta = pn;
					ct[Nct].px = px;
					ct[Nct].py = py;
					ct[Nct].pz = pz;
					ct[Nct].nx = p.Nx();
					ct[Nct].ny = p.Ny();
					ct[Nct].nz = p.Nz();
					ct[Nct].pa = &p;
					ct[Nct].ba = b;
					ct[Nct].nba = i;
					Nct++;
				}
			}
		}
		else{
			// pt ok seul
			if((fabs(pt) < p.Dt()/2) && (fabs(ps) > p.Ds()/2)){
				ps = ps/fabs(ps)*p.Ds()/2;
			}
			// ps ok seul
			if((fabs(pt) > p.Dt()/2) && (fabs(ps) < p.Ds()/2)){
				pt = pt/fabs(pt)*p.Dt()/2;
			}
			// aucun ok
			if((fabs(pt) > p.Dt()/2) && (fabs(ps) > p.Ds()/2)){
				ps = ps/fabs(ps)*p.Ds()/2;
				pt = pt/fabs(pt)*p.Dt()/2;
			}
			// Point du plan candidat au contact
			px = p.X() + ps*p.Sx() + pt*p.Tx();
			py = p.Y() + ps*p.Sy() + pt*p.Ty();
			pz = p.Z() + ps*p.Sz() + pt*p.Tz();

			double nnx = px-b->X();
			double nny = py-b->Y();
			double nnz = pz-b->Z();
			double nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
			if(nn <= b->SphereRadius(i)){
				ct[Nct].delta = nn - b->SphereRadius(i);
				ct[Nct].type = Contact::Type::BodyPlan;// 11;
				ct[Nct].px = px;
				ct[Nct].py = py;
				ct[Nct].pz = pz;
				ct[Nct].nx = nnx/nn;
				ct[Nct].ny = nny/nn;
				ct[Nct].nz = nnz/nn;
				ct[Nct].pa = &p;
				ct[Nct].ba = b;
				ct[Nct].nba = i;
				Nct++;
			}
		}
	}
}
