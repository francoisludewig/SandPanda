#include "../../Includes/Contact/ContactDetectorBodyVersusElbow.h"

#include "../../Includes/Solids/Elbow.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Contact/Contact.h"

void ContactDetectorBodyVersusElbow::Detect(Elbow & p, Body *b, Contact *ct, int & Nct) noexcept {
	double rl,D,pn,pt,ps,alphal;
	double cx,cy,cz,nx,ny,nz;
	for(int i = 0 ; i < b->SphereCount() ; i++){
		pn = (b->SphereX(i)-p.x)*p.nx + (b->SphereY(i)-p.y)*p.ny + (b->SphereZ(i)-p.z)*p.nz;
		pt = (b->SphereX(i)-p.x)*p.tx + (b->SphereY(i)-p.y)*p.ty + (b->SphereZ(i)-p.z)*p.tz;
		ps = (b->SphereX(i)-p.x)*p.sx + (b->SphereY(i)-p.y)*p.sy + (b->SphereZ(i)-p.z)*p.sz;
		rl = sqrt(pn*pn+pt*pt);

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
				cx = p.x + (pn*p.nx + pt*p.tx)/rl*p.R;
				cy = p.y + (pn*p.ny + pt*p.ty)/rl*p.R;
				cz = p.z + (pn*p.nz + pt*p.tz)/rl*p.R;
				D = sqrt(pow(b->X()-cx,2)+pow(b->Y()-cy,2)+pow(b->Z()-cz,2));
				if(p.r-(D+b->SphereRadius(i)) < 0){
					ct[Nct].delta =  p.r-(D+b->SphereRadius(i));
					ct[Nct].type = Contact::Type::BodyElbow;// 14;
					nx = -(cx - b->SphereX(i));
					ny = -(cy - b->SphereY(i));
					nz = -(cz - b->SphereZ(i));
					D = sqrt(nx*nx+ny*ny+nz*nz);
					ct[Nct].nx = nx/D;
					ct[Nct].ny = ny/D;
					ct[Nct].nz = nz/D;
					ct[Nct].px = b->SphereX(i) + b->SphereRadius(i)*ct[Nct].nx;
					ct[Nct].py = b->SphereY(i) + b->SphereRadius(i)*ct[Nct].ny;
					ct[Nct].pz = b->SphereZ(i) + b->SphereRadius(i)*ct[Nct].nz;
					ct[Nct].ba = b;
					ct[Nct].nba = i;
					ct[Nct].ew = &p;
					Nct++;
				}
			}
		}
	}
}
