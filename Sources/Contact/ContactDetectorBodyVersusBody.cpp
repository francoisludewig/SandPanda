#include "../../Includes/Contact/ContactDetectorBodyVersusBody.h"

#include "../../Includes/Solids/Body.h"
#include "../../Includes/Contact/Contact.h"

#include <cmath>

int ContactDetectorBodyVersusBody::Detect(Body *a, Body *b, Contact *ct, int &Nct, double ra, double rb) noexcept {
	int Nctfound = 0;
	double Nx,Ny,Nz,xc,yc,zc,norme;

	// Vecteur normal joignant les centres de masse
	Nx = a->X()-b->X();
	Ny = a->Y()-b->Y();
	Nz = a->Z()-b->Z();
	norme = sqrt(Nx*Nx+Ny*Ny+Nz*Nz);
	Nx = Nx/norme;
	Ny = Ny/norme;
	Nz = Nz/norme;

	// Point central du segment joignant les centres de masse
	xc = (ra*a->X()+rb*b->X())/(ra+rb);
	yc = (ra*a->Y()+rb*b->Y())/(ra+rb);
	zc = (ra*a->Z()+rb*b->Z())/(ra+rb);

	double deltaA[a->SphereCount()];
	double deltaB[b->SphereCount()];
	double dminA = 10,dmaxB = -10;
	double dmaxA = a->MaximumRadius();
	double dminB = -b->MaximumRadius();
	int Na = 0,Nb = 0;
	double Min,Max;
	//int Nct0 = Nct;
	double px,py,pz,Q,P2;

	// Liste des sph de a
	for(int i = 0 ; i < a->SphereCount() ; i++){
		deltaA[i] = (a->SphereX(i)-xc)*Nx+(a->SphereY(i)-yc)*Ny+(a->SphereZ(i)-zc)*Nz;
		if(deltaA[i]-a->SphereRadius(i) < dminA)dminA = deltaA[i]-a->SphereRadius(i);
		if(deltaA[i] < dmaxA)Na++;
	}
	// Liste des sph de b
	for(int i = 0 ; i < b->SphereCount() ; i++){
		deltaB[i] = (b->SphereX(i)-xc)*Nx+(b->SphereY(i)-yc)*Ny+(b->SphereZ(i)-zc)*Nz;
		if(deltaB[i]+b->SphereRadius(i) > dmaxB)dmaxB = deltaB[i]+b->SphereRadius(i);
		if(deltaB[i] > dminB)Nb++;
	}

	// Tolerence de securite
	dminA -= a->MaximumRadius();
	dminB -= b->MaximumRadius();
	dmaxA += a->MaximumRadius();
	dmaxB += b->MaximumRadius();

	if(dmaxB > dmaxA)
		Max = dmaxB;
	else
		Max = dmaxA;
	if(dminA < dminB)
		Min = dminA;
	else
		Min = dminB;


	int Ncontrol = 0;
	int Ntest = 0;

	for(int i = 0 ; i < a->SphereCount() ; i++){
		for(int j = 0 ; j < b->SphereCount() ; j++){
			Ncontrol++;
			if(deltaA[i] < Max && deltaB[j] < Max && deltaA[i] > Min && deltaB[j] > Min){
				Ntest++;
				//double px,py,pz,Q,P2;
				px = a->SphereX(i) - b->SphereX(j);
				py = a->SphereY(i) - b->SphereY(j);
				pz = a->SphereZ(i) - b->SphereZ(j);
				Q = (a->SphereRadius(i)+b->SphereRadius(j));
				P2 = px*px+py*py+pz*pz;
				if(P2 < Q*Q){

					P2 = sqrt(P2);
					ct[Nct].type = Contact::Type::BodyBody;// 20;
					ct[Nct].delta = P2-Q;

					ct[Nct].nx = px/P2;
					ct[Nct].ny = py/P2;
					ct[Nct].nz = pz/P2;

					ct[Nct].px = b->SphereX(j) + px/2.;
					ct[Nct].py = b->SphereY(j) + py/2.;
					ct[Nct].pz = b->SphereZ(j) + pz/2.;

					ct[Nct].ba = a;
					ct[Nct].nba = i;
					ct[Nct].bb = b;
					ct[Nct].nbb = j;
					Nct++;
					Nctfound++;
				}
			}
		}
	}
	if(Nctfound == 0)
		return 0;
	else
		return 1;
}
