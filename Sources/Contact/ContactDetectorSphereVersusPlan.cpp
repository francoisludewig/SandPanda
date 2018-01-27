#include "../../Includes/Contact/ContactDetectorSphereVersusPlan.h"

#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Plan.h"
#include "../../Includes/Contact/ContactDetectorBodyVersusPlan.h"

void ContactDetectorSphereVersusPlan::Detect(Plan & p, Sphere *b, Contact *ct, int & Nct) noexcept {
	if(b->Bodies() != -9) {
		ContactDetectorBodyVersusPlan::Detect(p,b->GetBody(),ct,Nct);
		return;
	}

	double px = b->X() - b->Radius()*p.Nx();
	double py = b->Y() - b->Radius()*p.Ny();
	double pz = b->Z() - b->Radius()*p.Nz();
	double pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
	double pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
	double ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
	if((fabs(pt) < p.Dt()/2) && (fabs(ps) < p.Ds()/2)){
		if( (pn <= 0.0) && (pn > -b->Radius())){
			ct[Nct].type = Contact::Type::SpherePlan;// 1;
			ct[Nct].delta = pn;
			ct[Nct].px = px;
			ct[Nct].py = py;
			ct[Nct].pz = pz;
			ct[Nct].nx = -p.Nx();
			ct[Nct].ny = -p.Ny();
			ct[Nct].nz = -p.Nz();
			p.Normal(&ct[Nct],b);
			ct[Nct].pa = &p;
			ct[Nct].sa = b;
			Nct++;
		}
		else{
			if(p.Sigma() != 0)
				b->Ct_pl(0);
		}
		if(p.InAndOut() == 1){
			px = b->X() + b->Radius()*p.Nx();
			py = b->Y() + b->Radius()*p.Ny();
			pz = b->Z() + b->Radius()*p.Nz();
			pn = (px-p.X())*(-p.Nx())+(py-p.Y())*(-p.Ny())+(pz-p.Z())*(-p.Nz());
			pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
			ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
			if( (pn <= 0.0) && (pn > -b->Radius())){
				ct[Nct].type = Contact::Type::SpherePlan;// 1;
				ct[Nct].delta = pn;
				ct[Nct].px = px;
				ct[Nct].py = py;
				ct[Nct].pz = pz;
				ct[Nct].nx = p.Nx();
				ct[Nct].ny = p.Ny();
				ct[Nct].nz = p.Nz();
				p.Normal(&ct[Nct],b);
				ct[Nct].pa = &p;
				ct[Nct].sa = b;
				Nct++;
			}
			else{
				if(p.Sigma() != 0)
					b->Ct_pl(0);
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
		if(nn <= b->Radius()){
			ct[Nct].delta = nn - b->Radius();
			ct[Nct].type = Contact::Type::SpherePlan;// 1;
			ct[Nct].px = px;
			ct[Nct].py = py;
			ct[Nct].pz = pz;
			ct[Nct].nx = nnx/nn;
			ct[Nct].ny = nny/nn;
			ct[Nct].nz = nnz/nn;
			ct[Nct].pa = &p;
			ct[Nct].sa = b;
			Nct++;
		}
	}
}
