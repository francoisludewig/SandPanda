#include "../../Includes/Contact/ContactDetectorSphereVersusSphere.h"

#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/Sphere.h"

#include "../../Includes/Contact/ContactDetectorSphereVersusBody.h"
#include "../../Includes/Contact/ContactDetectorBodyVersusBody.h"

#include <cmath>

int ContactDetectorSphereVersusSphere::Detect(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept {

	if(a->Bodies() != -9 && b->Bodies() == -9)
		return ContactDetectorSphereVersusBody::Detect(b, a->GetBody(),ct,Nct);

	if(a->Bodies() == -9 && b->Bodies() != -9)
		return ContactDetectorSphereVersusBody::Detect(a, b->GetBody(),ct,Nct);

	if(a->Bodies() != -9 && b->Bodies() != -9)
		return ContactDetectorBodyVersusBody::Detect(a->GetBody(), b->GetBody(),ct,Nct,a->Radius(),b->Radius());

	double px = a->X() - b->X();
	double py = a->Y() - b->Y();
	double pz = a->Z() - b->Z();
	double Q = (a->Radius()+b->Radius());
	double P2 = px*px+py*py+pz*pz;

	if(P2 < Q*Q){
		P2 = sqrt(P2);
		ct[Nct].type = Contact::Type::SphereSphere;// 0;
		ct[Nct].delta = P2-Q;
		ct[Nct].nx = px/P2;
		ct[Nct].ny = py/P2;
		ct[Nct].nz = pz/P2;

		ct[Nct].px = b->X() + px/2.;
		ct[Nct].py = b->Y() + py/2.;
		ct[Nct].pz = b->Z() + pz/2.;

		ct[Nct].sa = a;
		ct[Nct].sb = b;
		Nct++;
		return 1;
	}

	return 0;
}
