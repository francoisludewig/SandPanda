#include "../../Includes/Contact/ContactDetectorSphereVersusBody.h"

#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Body.h"

#include <cmath>

int ContactDetectorSphereVersusBody::Detect(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept {
	int Nctfound = 0;
	double Nx,Ny,Nz,xc,yc,zc,norme;
	// Vecteur normal joignant les centres de masse
	Nx = a->X()-b->X();
	Ny = a->Y()-b->Y();
	Nz = a->Z()-b->Z();
	norme = std::sqrt(Nx*Nx+Ny*Ny+Nz*Nz);
	Nx = Nx/norme;
	Ny = Ny/norme;
	Nz = Nz/norme;

	// Point central du segment joignant les centres de masse
	xc = a->X() - Nx*a->Radius();
	yc = a->Y() - Ny*a->Radius();
	zc = a->Z() - Nz*a->Radius();

	double delta[b->SphereCount()];
	// Liste des sph de b
	for(int i = b->SphereCount() ; i--;){
		delta[i] = (b->SphereX(i)-xc)*Nx+(b->SphereY(i)-yc)*Ny+(b->SphereZ(i)-zc)*Nz;
	}

	// Test Contact potentiel
	for(int i = b->SphereCount() ; i--;){
		if(delta[i] > -b->SphereRadius(i)){
			double px,py,pz,Q,P2;
			px = a->X() - b->SphereX(i);
			py = a->Y() - b->SphereY(i);
			pz = a->Z() - b->SphereZ(i);
			Q = (a->Radius()+b->SphereRadius(i));
			P2 = px*px+py*py+pz*pz;
			if(P2 < Q*Q){
				P2 = sqrt(P2);
				ct[Nct].type = Contact::Type::SphereBody;// 10;
				ct[Nct].delta = P2-Q;

				ct[Nct].nx = px/P2;
				ct[Nct].ny = py/P2;
				ct[Nct].nz = pz/P2;

				ct[Nct].px = b->SphereX(i) + px/2.;
				ct[Nct].py = b->SphereY(i) + py/2.;
				ct[Nct].pz = b->SphereZ(i) + pz/2.;

				ct[Nct].sa = a;
				ct[Nct].bb = b;
				ct[Nct].nbb = i;
				Nct++;
				Nctfound++;
			}
		}
	}

	if(Nctfound == 0)
		return 0;
	else
		return 1;
}
