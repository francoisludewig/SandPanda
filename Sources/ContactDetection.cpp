#include "../Includes/ContactDetection.h"
//#include <omp.h>

#include "../Includes/Velocity.h"
#include "../Includes/Gravity.h"
#include "../Includes/Plan.h"
#include "../Includes/PlanR.h"
#include "../Includes/Cone.h"
#include "../Includes/Elbow.h"
#include "../Includes/Sphere.h"
#include "../Includes/Body.h"
#include "../Includes/Contact.h"
#include "../Includes/Data.h"
#include "../Includes/HollowBall.h"

int ContactDetection::ContactSphBody(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept {
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

int ContactDetection::ContactSphBodyPeriodic(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept {
	int Nctfound = 0;
	double Nx,Ny,Nz,xc,yc,zc,norme;
	// Vecteur normal joignant les centres de masse
	Nx = a->X2()-b->X();
	Ny = a->Y2()-b->Y();
	Nz = a->Z2()-b->Z();
	norme = sqrt(Nx*Nx+Ny*Ny+Nz*Nz);
	Nx = Nx/norme;
	Ny = Ny/norme;
	Nz = Nz/norme;
	
	// Point central du segment joignant les centres de masse
	xc = a->X2() - Nx*a->Radius();
	yc = a->Y2() - Ny*a->Radius();
	zc = a->Z2() - Nz*a->Radius();
	
	double delta[b->SphereCount()];
	// Liste des sph de b
	for(int i = b->SphereCount() ; i--;){
		delta[i] = (b->SphereX(i)-xc)*Nx+(b->SphereY(i)-yc)*Ny+(b->SphereZ(i)-zc)*Nz;
	}
	
	// Test Contact potentiel
	for(int i = b->SphereCount() ; i--;){
		if(delta[i] > -b->SphereRadius(i)){
			double px,py,pz,Q,P2;
			px = a->X2() - b->SphereX(i);
			py = a->Y2() - b->SphereY(i);
			pz = a->Z2() - b->SphereZ(i);
			Q = (a->Radius()+b->SphereRadius(i));
			P2 = px*px+py*py+pz*pz;
			if(P2 < Q*Q){
				P2 = sqrt(P2);
				ct[Nct].type = Contact::Type::SphereBody;// 10;
				ct[Nct].delta = P2-(a->Radius()+b->SphereRadius(i));
				
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

int ContactDetection::ContactBodyBody(Body *a, Body *b, Contact *ct, int &Nct, double ra, double rb) noexcept {
	int Nctfound = 0;
	double Nx,Ny,Nz,xc,yc,zc,norme;
	//printf("Ct Body Body\n");
	
	
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

int ContactDetection::ContactSphSph(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept {
	int rtn = 0;
	if(a->Bodies() == -9 && b->Bodies() == -9){
		double px,py,pz,Q,P2;
		px = a->X() - b->X();
		py = a->Y() - b->Y();
		pz = a->Z() - b->Z();
		Q = (a->Radius()+b->Radius());
		P2 = px*px+py*py+pz*pz;
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
			rtn = 1;
		}
	}
	if(a->Bodies() != -9 && b->Bodies() == -9){
		rtn = ContactSphBody(b, a->GetBody(),ct,Nct);
	}
	if(a->Bodies() == -9 && b->Bodies() != -9){
		rtn = ContactSphBody(a, b->GetBody(),ct,Nct);
	}
	if(a->Bodies() != -9 && b->Bodies() != -9){
		 rtn = ContactBodyBody(a->GetBody(), b->GetBody(),ct,Nct,a->Radius(),b->Radius());
	}
	return rtn;
}

void ContactDetection::ContactSphSphPeriodic(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept {
	int rtn = 0;
	if(a->Num() != b->Num()){
		if((a->Bodies() == -9 && b->Bodies() == -9)){
			double px,py,pz,P2,Q;
			px = a->X2() - b->X();
			py = a->Y2() - b->Y();
			pz = a->Z2() - b->Z();
			P2 = px*px+py*py+pz*pz;
			Q = (a->Radius()+b->Radius());
			if(P2 < Q*Q){
				P2 = sqrt(P2);
				ct[Nct].type = Contact::Type::SphereSphere;// 0;
				ct[Nct].delta = P2-Q;
				ct[Nct].nx = px/P2;
				ct[Nct].ny = py/P2;
				ct[Nct].nz = pz/P2;
				ct[Nct].px = b->X() + px/2;
				ct[Nct].py = b->Y() + py/2;
				ct[Nct].pz = b->Z() + pz/2;
				ct[Nct].sa = a;
				ct[Nct].sb = b;
				Nct++;
			}
		}
		
		else{
			if(a->Bodies() != -9 && b->Bodies() == -9){
				rtn = ContactSphBodyPeriodic(b, a->GetBody(),ct,Nct);
			}
			if(a->Bodies() == -9 && b->Bodies() != -9){
				rtn = ContactSphBodyPeriodic(a, b->GetBody(),ct,Nct);
			}
			if(a->Bodies() != -9 && b->Bodies() != -9){
				double tpx,tpy,tpz;
				tpx = a->X();
				tpy = a->Y();
				tpz = a->Z();
				(a->GetBody())->X(a->X2());
				(a->GetBody())->Y(a->Y2());
				(a->GetBody())->Z(a->Z2());
				(a->GetBody())->UpDateLinkedSphereTp();
				
				rtn = ContactBodyBody(a->GetBody(), b->GetBody(),ct,Nct,a->Radius(),b->Radius());
				(a->GetBody())->X(tpx);
				(a->GetBody())->Y(tpy);
				(a->GetBody())->Z(tpz);
				(a->GetBody())->UpDateLinkedSphereTp();
				
			}
		}
	}
}

void ContactDetection::ContactBodyPlan(Plan & p, Body *b, Contact *ct, int & Nct) noexcept {
	double px,py,pz,pn,pt,ps,nn,nnx,nny,nnz;
	for(int i = 0 ; i < b->SphereCount() ; i++){
		px = b->SphereX(i) - b->SphereRadius(i)*p.Nx();
		py = b->SphereY(i) - b->SphereRadius(i)*p.Ny();
		pz = b->SphereZ(i) - b->SphereRadius(i)*p.Nz();
		pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
		pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
		ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
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
			
			nnx = px-b->X();
			nny = py-b->Y();
			nnz = pz-b->Z();
			nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
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

void ContactDetection::ContactSphPlan(Plan & p, Sphere *b, Contact *ct, int & Nct) noexcept {
	if(b->Bodies() == -9){
		double px,py,pz,pn,pt,ps,nn,nnx,nny,nnz;
		px = b->X() - b->Radius()*p.Nx();
		py = b->Y() - b->Radius()*p.Ny();
		pz = b->Z() - b->Radius()*p.Nz();
		pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
		pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
		ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
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
				//printf("n = (%e,%e,%e)\n", ct[Nct].nx, ct[Nct].ny, ct[Nct].nz);
				p.Normal(&ct[Nct],b);
				//if(p.Sigma() != 0)
				//	printf("n = (%e,%e,%e)\n\n", ct[Nct].nx, ct[Nct].ny, ct[Nct].nz);
				
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
			
			nnx = px-b->X();
			nny = py-b->Y();
			nnz = pz-b->Z();
			nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
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
	else
		ContactBodyPlan(p,b->GetBody(),ct,Nct);
}

void ContactDetection::ContactSphPlanPeriodic(Sphere *llist[], Plan & p, Plan & p2, Sphere *b, const double rmax) noexcept {
	double px,py,pz,pn,pt,ps;
	px = b->X();
	py = b->Y();
	pz = b->Z();
	pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
	pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
	ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
	if( (pn <= (b->Radius()+rmax)) && (pn > -2*b->Radius()) && (fabs(pt) < p.Dt()/2+b->Radius()) &&
	   (fabs(ps) < p.Ds()/2+b->Radius()) ){
		pt = (px-p2.X())*p2.Tx()+(py-p2.Y())*p2.Ty()+(pz-p2.Z())*p2.Tz();
		ps = (px-p2.X())*p2.Sx()+(py-p2.Y())*p2.Sy()+(pz-p2.Z())*p2.Sz();
		b->X2(p2.X() + (-pn)*p2.Nx() + pt*p2.Tx() + ps*p2.Sx());
		b->Y2(p2.Y() + (-pn)*p2.Ny() + pt*p2.Ty() + ps*p2.Sy());
		b->Z2(p2.Z() + (-pn)*p2.Nz() + pt*p2.Tz() + ps*p2.Sz());
		llist[p.ListCount()] = b;
		p.List(p.ListCount(), b->Num());
		p.ListCount(p.ListCount() + 1);
	}
}

void ContactDetection::ContactBodyPlanR(PlanR & p, Body *b, Contact *ct, int & Nct) noexcept {
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

void ContactDetection::ContactSphPlanR(PlanR & p, Sphere *b, Contact *ct, int & Nct) noexcept {
	if(b->Bodies() == -9){
		double px,py,pz,pn,pt,ps;
		px = b->X() - b->Radius()*p.Nx();
		py = b->Y() - b->Radius()*p.Ny();
		pz = b->Z() - b->Radius()*p.Nz();
		pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
		pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
		ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
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
	else
		ContactBodyPlanR(p,b->GetBody(),ct,Nct);
}

void ContactDetection::ContactBodyCone(Cone & p, Body *b, Contact *ct, int & Nct) noexcept {
	double px,py,pz,pn,pt,ps,N,Tx,Ty,Tz,Nx,Ny,Nz,delta2,Qn,Qt,dr,h;
	double nnx,nny,nnz,nn;
	double a,bb,Y;
	for(int i = 0 ; i < b->SphereCount() ; i++){
		px = b->SphereX(i);
		py = b->SphereY(i);
		pz = b->SphereZ(i);
		pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
		pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
		ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
		
		if(fabs(pn) <= p.Height()/2 + b->SphereRadius(i)){
			N = sqrt(pt*pt+ps*ps);
			Tx = (pt*p.Tx() + ps*p.Sx())/N;
			Ty = (pt*p.Ty() + ps*p.Sy())/N;
			Tz = (pt*p.Tz() + ps*p.Sz())/N;
			Nx = p.Nx();
			Ny = p.Ny();
			Nz = p.Nz();
			dr = p.BottomRadius() -  p.TopRadius();
			h = p.Height();
			if(dr == 0){
				if(fabs(pn) <= p.Height()/2){
					if(fabs(N-p.BottomRadius()) <= b->SphereRadius(i)){
						px = p.X() + pn*Nx + p.BottomRadius()*Tx;
						py = p.Y() + pn*Ny + p.BottomRadius()*Ty;
						pz = p.Z() + pn*Nz + p.BottomRadius()*Tz;
						ct[Nct].delta = fabs(N-p.BottomRadius()) - b->SphereRadius(i);
						ct[Nct].type = Contact::Type::BodyCone;// 13;
						ct[Nct].px = px;
						ct[Nct].py = py;
						ct[Nct].pz = pz;
						ct[Nct].nx = Tx;
						ct[Nct].ny = Ty;
						ct[Nct].nz = Tz;
						ct[Nct].ba = b;
						ct[Nct].nba = i;
						ct[Nct].cn = &p;
						Nct++;
					}
				}
				else{
					pn = pn/fabs(pn)*p.Height()/2.;
					px = p.X() + pn*Nx + p.BottomRadius()*Tx;
					py = p.Y() + pn*Ny + p.BottomRadius()*Ty;
					pz = p.Z() + pn*Nz + p.BottomRadius()*Tz;
					nnx = px-b->SphereX(i);
					nny = py-b->SphereY(i);
					nnz = pz-b->SphereZ(i);
					nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
					if(nn < b->SphereRadius(i)){
						ct[Nct].delta = nn - b->SphereRadius(i);
						ct[Nct].type = Contact::Type::BodyCone;// 13;
						ct[Nct].px = px;
						ct[Nct].py = py;
						ct[Nct].pz = pz;
						ct[Nct].nx = nnx/nn;
						ct[Nct].ny = nny/nn;
						ct[Nct].nz = nnz/nn;
						ct[Nct].ba = b;
						ct[Nct].nba = i;
						ct[Nct].cn = &p;
						Nct++;
					}
				}
			}
			else{
				a = -h/dr;
				bb = h*(p.BottomRadius()/dr-0.5);
				Y = sqrt(dr*dr+h*h);
				delta2 = -(N - pn/a + bb/a)*h/Y;
				if(fabs(delta2) <= b->SphereRadius(i)){
					Qn = pn + delta2*dr/Y;
					Qt = N + delta2*h/Y;
					px = p.X() + Qn*Nx + Qt*Tx;
					py = p.Y() + Qn*Ny + Qt*Ty;
					pz = p.Z() + Qn*Nz + Qt*Tz;
					if(Qn < h/2. && Qn > -h/2.){
						ct[Nct].delta = fabs(delta2) - b->SphereRadius(i);
						ct[Nct].type = Contact::Type::BodyCone;// 13;
						ct[Nct].px = px;
						ct[Nct].py = py;
						ct[Nct].pz = pz;
						ct[Nct].nx = (px-b->SphereX(i))/fabs(delta2);
						ct[Nct].ny = (py-b->SphereY(i))/fabs(delta2);
						ct[Nct].nz = (pz-b->SphereZ(i))/fabs(delta2);
						ct[Nct].ba = b;
						ct[Nct].nba = i;
						ct[Nct].cn = &p;
						Nct++;
					}
					else{
						if(Qn >= h/2.){
							Qn = h/2.;
							Qt = p.TopRadius();
						}
						else{
							Qn = -h/2.;
							Qt = p.BottomRadius();
						}
						px = p.X() + Qn*Nx + Qt*Tx;
						py = p.Y() + Qn*Ny + Qt*Ty;
						pz = p.Z() + Qn*Nz + Qt*Tz;
						nnx = px-b->SphereX(i);
						nny = py-b->SphereY(i);
						nnz = pz-b->SphereZ(i);
						nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
						if(nn <= b->SphereRadius(i)){
							ct[Nct].delta = nn - b->SphereRadius(i);
							ct[Nct].type = Contact::Type::BodyCone;// 13;
							ct[Nct].px = px;
							ct[Nct].py = py;
							ct[Nct].pz = pz;
							ct[Nct].nx = nnx/nn;
							ct[Nct].ny = nny/nn;
							ct[Nct].nz = nnz/nn;
							ct[Nct].ba = b;
							ct[Nct].nba = i;
							ct[Nct].cn = &p;
							Nct++;
						}
					}
				}
			}
		}
	}
}

void ContactDetection::ContactSphCone(Cone & p, Sphere *b, Contact *ct, int & Nct) noexcept {
	if(b->Bodies() == -9){
		double px,py,pz,pn,pt,ps,N,Tx,Ty,Tz,Nx,Ny,Nz,delta2,Qn,Qt,dr,h;
		double nnx,nny,nnz,nn;
		double a,bb,Y;
		px = b->X();
		py = b->Y();
		pz = b->Z();
		pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
		pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
		ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();
		
		if(fabs(pn) <= p.Height()/2 + b->Radius()){
			N = sqrt(pt*pt+ps*ps);
			Tx = (pt*p.Tx() + ps*p.Sx())/N;
			Ty = (pt*p.Ty() + ps*p.Sy())/N;
			Tz = (pt*p.Tz() + ps*p.Sz())/N;
			Nx = p.Nx();
			Ny = p.Ny();
			Nz = p.Nz();
			dr = p.BottomRadius() -  p.TopRadius();
			h = p.Height();
			
			if(dr == 0){
				if(fabs(pn) <= p.Height()/2){
					if(fabs(N-p.BottomRadius()) <= b->Radius()){
						px = p.X() + pn*Nx + p.BottomRadius()*Tx;
						py = p.Y() + pn*Ny + p.BottomRadius()*Ty;
						pz = p.Z() + pn*Nz + p.BottomRadius()*Tz;
						ct[Nct].delta = fabs(N-p.BottomRadius()) - b->Radius();
						ct[Nct].type = Contact::Type::SphereCone;// 3;
						ct[Nct].px = px;
						ct[Nct].py = py;
						ct[Nct].pz = pz;
						if(p.In() == 0){
							ct[Nct].nx = Tx;
							ct[Nct].ny = Ty;
							ct[Nct].nz = Tz;
						}
						else{
							ct[Nct].nx = -Tx;
							ct[Nct].ny = -Ty;
							ct[Nct].nz = -Tz;
						}
						ct[Nct].sa = b;
						ct[Nct].cn = &p;
						Nct++;
					}
				}
				else{
					pn = pn/fabs(pn)*p.Height()/2.;
					px = p.X() + pn*Nx + p.BottomRadius()*Tx;
					py = p.Y() + pn*Ny + p.BottomRadius()*Ty;
					pz = p.Z() + pn*Nz + p.BottomRadius()*Tz;
					nnx = px-b->X();
					nny = py-b->Y();
					nnz = pz-b->Z();
					nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
					if(nn < b->Radius()){
						ct[Nct].delta = nn - b->Radius();
						ct[Nct].type = Contact::Type::SphereCone;// 3;
						ct[Nct].px = px;
						ct[Nct].py = py;
						ct[Nct].pz = pz;
						ct[Nct].nx = nnx/nn;
						ct[Nct].ny = nny/nn;
						ct[Nct].nz = nnz/nn;
						ct[Nct].sa = b;
						ct[Nct].cn = &p;
						Nct++;
					}
				}
			}
			else{
				
				a = -h/dr;
				bb = h*(p.BottomRadius()/dr-0.5);
				Y = sqrt(dr*dr+h*h);
				delta2 = -(N - pn/a + bb/a)*h/Y;
				
				if(fabs(delta2) <= b->Radius()){
					Qn = pn + delta2*dr/Y;
					Qt = N + delta2*h/Y;
					px = p.X() + Qn*Nx + Qt*Tx;
					py = p.Y() + Qn*Ny + Qt*Ty;
					pz = p.Z() + Qn*Nz + Qt*Tz;
					if(Qn < h/2. && Qn > -h/2.){
						ct[Nct].delta = fabs(delta2) - b->Radius();
						ct[Nct].type = Contact::Type::SphereCone;// 3;
						ct[Nct].px = px;
						ct[Nct].py = py;
						ct[Nct].pz = pz;
						nnx = (px-b->X());
						nny = (py-b->Y());
						nnz = (pz-b->Z());
						ct[Nct].nx = (px-b->X())/fabs(delta2);
						ct[Nct].ny = (py-b->Y())/fabs(delta2);
						ct[Nct].nz = (pz-b->Z())/fabs(delta2);
						ct[Nct].sa = b;
						ct[Nct].cn = &p;
						Nct++;
					}
					else{
						if(Qn >= h/2.){
							Qn = h/2.;
							Qt = p.TopRadius();
						}
						else{
							Qn = -h/2.;
							Qt = p.BottomRadius();
						}
						px = p.X() + Qn*Nx + Qt*Tx;
						py = p.Y() + Qn*Ny + Qt*Ty;
						pz = p.Z() + Qn*Nz + Qt*Tz;
						nnx = px-b->X();
						nny = py-b->Y();
						nnz = pz-b->Z();
						nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
						if(nn <= b->Radius()){
							ct[Nct].delta = nn - b->Radius();
							ct[Nct].type = Contact::Type::SphereCone;// 3;
							ct[Nct].px = px;
							ct[Nct].py = py;
							ct[Nct].pz = pz;
							ct[Nct].nx = nnx/nn;
							ct[Nct].ny = nny/nn;
							ct[Nct].nz = nnz/nn;
							ct[Nct].sa = b;
							ct[Nct].cn = &p;
							Nct++;
						}
					}
				}
			}
		}
	}
	else
		ContactBodyCone(p,b->GetBody(),ct,Nct);
}

void ContactDetection::ContactBodyElbow(Elbow & p, Body *b, Contact *ct, int & Nct) noexcept {
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

void ContactDetection::ContactSphElbow(Elbow & p, Sphere *b, Contact *ct, int & Nct) noexcept {
	if(b->Bodies() == -9){
		double rl,D,pn,pt,ps,alphal;
		double cx,cy,cz,nx,ny,nz;
		
		pn = (b->X()-p.x)*p.nx + (b->Y()-p.y)*p.ny + (b->Z()-p.z)*p.nz;
		pt = (b->X()-p.x)*p.tx + (b->Y()-p.y)*p.ty + (b->Z()-p.z)*p.tz;
		ps = (b->X()-p.x)*p.sx + (b->Y()-p.y)*p.sy + (b->Z()-p.z)*p.sz;
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
				if(p.r-(D+b->Radius()) < 0){
					ct[Nct].delta =  p.r-(D+b->Radius());
					ct[Nct].type = Contact::Type::SphereElbow;// 4;
					nx = -(cx - b->X());
					ny = -(cy - b->Y());
					nz = -(cz - b->Z());
					D = sqrt(nx*nx+ny*ny+nz*nz);
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
	else
		ContactBodyElbow(p,b->GetBody(),ct,Nct);
}

void ContactDetection::linkedCell(std::vector<Sphere> & sph, const int Nsph, const Data *dat, Sphere *cell[]) noexcept {
	int x,y,z;
	// Initialisation du tableau tdl
	for(int i = Nsph ; i--;)
		sph[i].TDL(nullptr);
	
	// Initialisation du tableau Cell
	for(int i = dat->Ncellmax ; i--;)
		cell[i] = nullptr;
	
	// Classement des grains dans les cellules
	for(int i = Nsph ; i--;){
		if(sph[i].HollowballNum() == -9){
			x = (int)((sph[i].X() - dat->xmin)/dat->ax);
			y = (int)((sph[i].Y() - dat->ymin)/dat->ay);
			z = (int)((sph[i].Z() - dat->zmin)/dat->az);
			if(x < dat->Nx && y < dat->Ny && z < dat->Nz && x >= 0 && y >= 0 && z >= 0){
				//printf("Found it\n");
				//printf("%d in %d\n",i,x*dat->Ny*dat->Nz+y*dat->Nz+z);
				sph[i].TDL(cell[x*dat->Ny*dat->Nz+y*dat->Nz+z]);
				cell[x*dat->Ny*dat->Nz+y*dat->Nz+z] = &sph[i];
			}
		}
	}
}

void ContactDetection::listCellForPlan(Data *dat, std::vector<Plan> & pl, int & Npl, Gravity& gt) noexcept {
	int a,b;
	int i,j,k;
	int I,num;
	double x,y,z,pn,pt,ps;
	double dn,dt,ds;
	double ox,oy,oz;
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	
	int doublon;
	int *localCell;
	int localNcell;
	
	double Vmax,time,dtime,dist;
	int Nts;
	localCell =(int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
	
	for(a = 0 ; a < Npl ; a++){
		if(pl[a].GetForce() == 0){
			Vmax = pl[a].Vmax();
			if(Vmax < pl[a].Wmax()*pl[a].Dt())Vmax = pl[a].Wmax()*pl[a].Dt();
			if(Vmax < pl[a].Wmax()*pl[a].Ds())Vmax = pl[a].Wmax()*pl[a].Ds();
			
			time = pl[a].Delay();
			
			dist = dat->ax/2;
			if(dist > dat->ay/2)dist = dat->ay/2;
			if(dist > dat->az/2)dist = dat->az/2;
			
			if(Vmax != 0)
				dtime = dist/Vmax;
			else
				dtime = 0.0;
			
			if(time == 10000.0)time = dat->Total;
			
			if(time != 0)
				Nts = (int)(time/dtime)+1;
			else
				Nts = 0;
			
			
			localNcell = 0;
			for(i = dat->Nx*dat->Ny*dat->Nz ; i--;){
				localCell[i] = -9;
			}
			
			printf("Nts(%d) = %d\n",a,Nts);
			for(b = 0 ; b <= Nts ; b++){
				if(b != 0){
					pl[a].UpDateVelocity(b*dtime,dtime,gt);
					pl[a].Move(dtime);
				}
				ox = pl[a].X();
				oy = pl[a].Y();
				oz = pl[a].Z();
				nx = pl[a].Nx();
				ny = pl[a].Ny();
				nz = pl[a].Nz();
				tx = pl[a].Tx();
				ty = pl[a].Ty();
				tz = pl[a].Tz();
				sx = pl[a].Sx();
				sy = pl[a].Sy();
				sz = pl[a].Sz();
				
				//printf("at b = %d => t = %e => z = %e\n",b,b*dtime,oz);
				dn = sqrt(dat->ax*dat->ax+dat->ay*dat->ay+dat->az*dat->az);
				dt = dn;
				ds = dn;
				for(i = 0 ; i < dat->Nx ; i++){
					for(j = 0 ; j < dat->Ny ; j++){
						for(k = 0 ; k < dat->Nz ; k++){
							x = dat->xmin + dat->ax*(i+0.5);
							y = dat->ymin + dat->ay*(j+0.5);
							z = dat->zmin + dat->az*(k+0.5);
							pn = (x-ox)*nx+(y-oy)*ny+(z-oz)*nz;
							pt = (x-ox)*tx+(y-oy)*ty+(z-oz)*tz;
							ps = (x-ox)*sx+(y-oy)*sy+(z-oz)*sz;
							if( (fabs(pt) < (pl[a].Dt()/2+dt)) &&
							   (fabs(ps) < (pl[a].Ds()/2+ds)) &&
							   (pn <= dn) && pn > -dn){
								// Control de doublon
								doublon = 0;
								num = (i*dat->Ny*dat->Nz+j*dat->Nz+k);
								for(I = 0 ; I < localNcell ; I++){
									if(num == localCell[I]){
										doublon++;
									}
								}
								if(doublon == 0){
									localCell[localNcell] = num;
									localNcell++;
								}
							}
						}
					}
				}
			}
			pl[a].Cell = (int*)malloc(localNcell*sizeof(int));
			pl[a].NCell = localNcell;
			for(i = 0 ; i < localNcell ; i++){
				pl[a].Cell[i] = localCell[i];
			}
			printf("Ncell(%d pl) = %d\n",a,pl[a].NCell);
		}
		else{
			// Si une force est appliquee toutes les cellules sont considerees
			pl[a].Cell = (int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
			pl[a].NCell = dat->Nx*dat->Ny*dat->Nz;
			printf("Ncell(%d pl) = %d\n",a,pl[a].NCell);
			for(i = 0 ; i < dat->Nx*dat->Ny*dat->Nz ; i++){
				pl[a].Cell[i] = i;
			}
		}
		
	}
	free(localCell);
}

void ContactDetection::listCellForPlanR(Data *dat, std::vector<PlanR> & plr, int & Nplr, Gravity& gt) noexcept {
	int a,b;
	int i,j,k;
	int I,num;
	double x,y,z,pn,pt,ps;
	double dn;
	double ox,oy,oz;
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	
	int doublon;
	int *localCell;
	int localNcell;
	
	double Vmax,time,dtime,dist;
	int Nts;
	
	localCell =(int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
	
	for(a = 0 ;  a < Nplr ; a++){
		
		if(plr[a].GetForce() == 0){
			
			Vmax = plr[a].Vmax();
			
			if(Vmax < plr[a].Wmax()*plr[a].Radius())Vmax = plr[a].Wmax()*plr[a].Radius();
			time = plr[a].Delay();
			
			dist = dat->ax/2;
			if(dist > dat->ay/2)dist = dat->ay/2;
			if(dist > dat->az/2)dist = dat->az/2;
			if(Vmax != 0)
				dtime = dist/Vmax;
			else
				dtime = 0.0;
			
			if(time == 10000.0)time = dat->Total;
			
			if(time != 0)
				Nts = (int)(time/dtime)+1;
			else
				Nts = 0;
			
			localNcell = 0;
			for(i = dat->Nx*dat->Ny*dat->Nz ; i--;){
				localCell[i] = -9;
			}
			
			for(b = 0 ; b <= Nts ; b++){
				if(b != 0){
					plr[a].UpDateVelocity(b*dtime,dtime,gt);
					plr[a].Move(dtime);
				}
				ox = plr[a].X();
				oy = plr[a].Y();
				oz = plr[a].Z();
				nx = plr[a].Nx();
				ny = plr[a].Ny();
				nz = plr[a].Nz();
				tx = plr[a].Tx();
				ty = plr[a].Ty();
				tz = plr[a].Tz();
				sx = plr[a].Sx();
				sy = plr[a].Sy();
				sz = plr[a].Sz();
				dn = sqrt(dat->ax*dat->ax+dat->ay*dat->ay+dat->az*dat->az);
				
				for(i = 0 ; i < dat->Nx ; i++){
					for(j = 0 ; j < dat->Ny ; j++){
						for(k = 0 ; k < dat->Nz ; k++){
							x = dat->xmin + dat->ax*(i+0.5);
							y = dat->ymin + dat->ay*(j+0.5);
							z = dat->zmin + dat->az*(k+0.5);
							pn = (x-ox)*nx+(y-oy)*ny+(z-oz)*nz;
							pt = (x-ox)*tx+(y-oy)*ty+(z-oz)*tz;
							ps = (x-ox)*sx+(y-oy)*sy+(z-oz)*sz;
							if( (sqrt(pt*pt+ps*ps) < plr[a].Radius()+2*dn) &&
							   (pn >= -dn) && (pn < 2*dn)){
								// Control de doublon
								doublon = 0;
								num = (i*dat->Ny*dat->Nz+j*dat->Nz+k);
								for(I = 0 ; I < localNcell ; I++){
									if(num == localCell[I]){
										doublon++;
									}
								}
								if(doublon == 0){
									localCell[localNcell] = num;
									localNcell++;
								}
							}
						}
					}
				}
			}
			
			plr[a].Cell = (int*)malloc(localNcell*sizeof(int));
			plr[a].NCell = localNcell;
			
			printf("Ncell(%d plr) = %d\n",a,plr[a].NCell);
			for(i = 0 ; i < localNcell ; i++){
				plr[a].Cell[i] = localCell[i];
			}
		}
		else{
			// Si une force est appliquee toutes les cellules sont considerees
			plr[a].Cell = (int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
			plr[a].NCell = dat->Nx*dat->Ny*dat->Nz;
			printf("Ncell(%d plr) = %d\n",a,plr[a].NCell);
			for(i = 0 ; i < dat->Nx*dat->Ny*dat->Nz ; i++){
				plr[a].Cell[i] = i;
			}
			
		}
	}
	free(localCell);
}

void ContactDetection::listCellForCone(Data *dat, std::vector<Cone> & co, int & Nco, Gravity& gt) noexcept {
	int numCone;
	int i,j,k;
	int I,num;
	double px,py,pz,pn,pt,ps;
	double dn;
	double ox,oy,oz;
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	int doublon;
	int *localCell;
	int localNcell;
	double N,dr,h,b,Y,delta2,beta,a;
	
	double Vmax,time,dtime,dist;
	int Nts,b2;
	
	
	localCell =(int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
	
	for(numCone = 0 ;  numCone < Nco ; numCone++){
		if(co[numCone].GetForce() == 0){
			Vmax = co[numCone].Vmax();
			if(Vmax < co[numCone].Wmax()*co[numCone].Height()/2.)Vmax = co[numCone].Wmax()*co[numCone].Height()/2.;
			time = co[numCone].Delay();
			
			dist = dat->ax/2;
			if(dist > dat->ay/2)dist = dat->ay/2;
			if(dist > dat->az/2)dist = dat->az/2;
			
			if(Vmax != 0)
				dtime = dist/Vmax;
			else
				dtime = 0.0;
			if(time == 10000.0)time = dat->Total;
			
			if(time != 0)
				Nts = (int)(time/dtime)+1;
			else
				Nts = 0;
			
			
			
			localNcell = 0;
			for(i = dat->Nx*dat->Ny*dat->Nz ; i--;){
				localCell[i] = -9;
			}
			
			printf("Nts(%d) = %d\n",numCone,Nts);
			
			for(b2 = 0 ; b2 <= Nts ; b2++){
				if(b2 != 0){
					co[numCone].UpDateVelocity(b2*dtime,dtime,gt);
					co[numCone].Move(dtime);
				}
				ox = co[numCone].X();
				oy = co[numCone].Y();
				oz = co[numCone].Z();
				nx = co[numCone].Nx();
				ny = co[numCone].Ny();
				nz = co[numCone].Nz();
				tx = co[numCone].Tx();
				ty = co[numCone].Ty();
				tz = co[numCone].Tz();
				sx = co[numCone].Sx();
				sy = co[numCone].Sy();
				sz = co[numCone].Sz();
				if(dat->Nx > 1)
					dn = sqrt(dat->ax*dat->ax+dat->ay*dat->ay+dat->az*dat->az);
				else{
					dn = sqrt(dat->ay*dat->ay+dat->az*dat->az);
					
				}
				for(i = 0 ; i < dat->Nx ; i++){
					for(j = 0 ; j < dat->Ny ; j++){
						for(k = 0 ; k < dat->Nz ; k++){
							px = dat->xmin + dat->ax*(i+0.5);
							py = dat->ymin + dat->ay*(j+0.5);
							pz = dat->zmin + dat->az*(k+0.5);
							pn = (px-ox)*nx+(py-oy)*ny+(pz-oz)*nz;
							pt = (px-ox)*tx+(py-oy)*ty+(pz-oz)*tz;
							ps = (px-ox)*sx+(py-oy)*sy+(pz-oz)*sz;
							
							if(fabs(pn) <= co[numCone].Height()/2+dn){
								N = sqrt(pt*pt+ps*ps);
								if(ps >= 0)
									beta = acos(pt/N);
								else
									beta = 2*M_PI-acos(pt/N);
								
								// 3D -> 2D
								dr = co[numCone].BottomRadius() -  co[numCone].TopRadius();
								h = co[numCone].Height();
								if(dr == 0){
									if(fabs(N-co[numCone].BottomRadius()) <= dn*2){
										//Cellule validee
										// Control de doublon
										doublon = 0;
										num = (i*dat->Ny*dat->Nz+j*dat->Nz+k);
										for(I = 0 ; I < localNcell ; I++){
											if(num == localCell[I]){
												doublon++;
											}
										}
										if(doublon == 0){
											localCell[localNcell] = num;
											localNcell++;
										}
									}
								}
								else{
									a = -h/dr;
									b = h*(co[numCone].BottomRadius()/dr-0.5);
									Y = sqrt(dr*dr+h*h);
									delta2 = (N - pn/a + b/a) / (dr/a/Y-h/Y);
									if(fabs(delta2) <= 2*dn){
										//Cellule validee
										// Control de doublon
										doublon = 0;
										num = (i*dat->Ny*dat->Nz+j*dat->Nz+k);
										for(I = 0 ; I < localNcell ; I++){
											if(num == localCell[I]){
												doublon++;
											}
										}
										if(doublon == 0){
											localCell[localNcell] = num;
											localNcell++;
										}
									}
								}
							}
						}
					}
					
				}
			}
			
			co[numCone].Cell = (int*)malloc(localNcell*sizeof(int));
			co[numCone].NCell = localNcell;
			for(i = 0 ; i < localNcell ; i++){
				co[numCone].Cell[i] = localCell[i];
			}
			printf("Ncell(%d co) = %d\n",numCone,co[numCone].NCell);
		}
		else{
			// Si une force est appliquee toutes les cellules sont considerees
			co[numCone].Cell = (int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
			co[numCone].NCell = dat->Nx*dat->Ny*dat->Nz;
			printf("Ncell(%d plr) = %d\n",numCone,co[numCone].NCell);
			for(i = 0 ; i < dat->Nx*dat->Ny*dat->Nz ; i++){
				co[numCone].Cell[i] = i;
			}
			
		}
	}
	free(localCell);
	
}

void ContactDetection::listCellForElbow(Data *dat, std::vector<Elbow> & el, int & Nelb) noexcept {
	int numEl;
	int i,j,k;
	int I,num;
	double px,py,pz,pn,pt,ps;
	double dn;
	double ox,oy,oz;
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	
	double Sox,Soy,Soz;
	double Snx,Sny,Snz;
	double Stx,Sty,Stz;
	double Ssx,Ssy,Ssz;
	
	int doublon;
	int *localCell;
	int localNcell;
	
	double Vmax,time,dtime,dist;
	int Nts,b;
	double rl, alphal,cx,cy,cz,D;
	
	localCell =(int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
	
	for(numEl = 0 ;  numEl < Nelb ; numEl++){
		Vmax = el[numEl].Vmax();
		if(Vmax < el[numEl].Wmax()*(el[numEl].r+el[numEl].R))Vmax = el[numEl].Wmax()*(el[numEl].r+el[numEl].R);
		time = el[numEl].Delay();
		
		dist = dat->ax/2;
		if(dist > dat->ay/2)dist = dat->ay/2;
		if(dist > dat->az/2)dist = dat->az/2;
		if(Vmax != 0)
			dtime = dist/Vmax;
		else
			dtime = 0.0;
		if(time == 10000.0)time = dat->Total;
		
		if(time != 0)
			Nts = (int)(time/dtime)+1;
		else
			Nts = 0;
		
		localNcell = 0;
		for(i = dat->Nx*dat->Ny*dat->Nz ; i--;){
			localCell[i] = -9;
		}
		
		Sox = el[numEl].x;
		Soy = el[numEl].y;
		Soz = el[numEl].z;
		Snx = el[numEl].nx;
		Sny = el[numEl].ny;
		Snz = el[numEl].nz;
		Stx = el[numEl].tx;
		Sty = el[numEl].ty;
		Stz = el[numEl].tz;
		Ssx = el[numEl].sx;
		Ssy = el[numEl].sy;
		Ssz = el[numEl].sz;
		
		for(b = 0 ; b <= Nts ; b++){
			if(b != 0){
				el[numEl].Move(b*dtime,dtime);
			}
			ox = el[numEl].x;
			oy = el[numEl].y;
			oz = el[numEl].z;
			nx = el[numEl].nx;
			ny = el[numEl].ny;
			nz = el[numEl].nz;
			tx = el[numEl].tx;
			ty = el[numEl].ty;
			tz = el[numEl].tz;
			sx = el[numEl].sx;
			sy = el[numEl].sy;
			sz = el[numEl].sz;
			dn = sqrt(dat->ax*dat->ax+dat->ay*dat->ay+dat->az*dat->az);
			for(i = 0 ; i < dat->Nx ; i++){
				for(j = 0 ; j < dat->Ny ; j++){
					for(k = 0 ; k < dat->Nz ; k++){
						px = dat->xmin + dat->ax*(i+0.5);
						py = dat->ymin + dat->ay*(j+0.5);
						pz = dat->zmin + dat->az*(k+0.5);
						pn = (px-ox)*nx + (py-oy)*ny + (pz-oz)*nz;
						pt = (px-ox)*tx + (py-oy)*ty + (pz-oz)*tz;
						ps = (px-ox)*sx + (py-oy)*sy + (pz-oz)*sz;
						
						rl = sqrt(pn*pn+pt*pt);
						if(rl >= el[numEl].R-el[numEl].r-2*dn && rl <= el[numEl].R+el[numEl].r+2*dn && fabs(ps) <= el[numEl].r+2*dn){
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
							if((alphal < M_PI && alphal <= el[numEl].alpha + 2*dn/rl) || (alphal >= M_PI && alphal-2*M_PI >= -2*dn/rl)){
								cx = ox + (pn*nx + pt*tx)/rl*el[numEl].R;
								cy = oy + (pn*ny + pt*ty)/rl*el[numEl].R;
								cz = oz + (pn*nz + pt*tz)/rl*el[numEl].R;
								D = sqrt(pow(px-cx,2)+pow(py-cy,2)+pow(pz-cz,2));
								if(el[numEl].r-(D+2*dn) < 0){
									//Cellule validee
									// Control de doublon
									doublon = 0;
									num = (i*dat->Ny*dat->Nz+j*dat->Nz+k);
									for(I = 0 ; I < localNcell ; I++){
										if(num == localCell[I]){
											doublon++;
										}
									}
									if(doublon == 0){
										localCell[localNcell] = num;
										localNcell++;
									}
								}
							}
						}
					}
				}
			}
		}
		
		el[numEl].Cell = (int*)malloc(localNcell*sizeof(int));
		el[numEl].NCell = localNcell;
		for(i = 0 ; i < localNcell ; i++){
			el[numEl].Cell[i] = localCell[i];
		}
		printf("Ncell(%d el) = %d\n",numEl,el[numEl].NCell);
		el[numEl].x = Sox;
		el[numEl].y = Soy;
		el[numEl].z = Soz;
		el[numEl].nx = Snx;
		el[numEl].ny = Sny;
		el[numEl].nz = Snz;
		el[numEl].tx = Stx;
		el[numEl].ty = Sty;
		el[numEl].tz = Stz;
		el[numEl].sx = Ssx;
		el[numEl].sy = Ssy;
		el[numEl].sz = Ssz;
	}
	free(localCell);
}

void ContactDetection::sphContactAll(const int & Nsph, std::vector<Sphere> & sph, Contact *ctl, int & Nctl) noexcept {
	int i,j;
	for(i = 0 ; i < Nsph ; i++){
		for(j = i+1 ; j < Nsph ; j++){
			ContactSphSph(&sph[i],&sph[j],ctl,Nctl);
		}
	}
}

void ContactDetection::sphContact(const int Nx0, const int Nx1, const int Nx, const int Ny0, const int Ny1, const int Ny, const int Nz, Contact *ctl, int & Nctl, Sphere **cell) noexcept {
	int l,m;
	Sphere *cand[5000],*anta;
	int i,j,k,Ncand;
	int num;
	for(i = Nx0 ; i < Nx1 ; i++){
		for(j = Ny0 ; j < Ny1 ; j++){
			for(k = 0 ; k < Nz ; k++){
				// num = i*Ny*Nz+j*Nz+k
				num = i*Ny*Nz+j*Nz+k;
				Ncand = 0;
				
				//printf("(%d,%d,%d) => %d\n",i,j,k,num);
				
				if((cand[Ncand] = cell[num]) != nullptr){
					//printf("(%d,%d,%d) => %d\n",i,j,k,num);
					do{
						//cand[Ncand]->affiche();
						Ncand++;
					}while((cand[Ncand] = cand[Ncand-1]->TDL()) != nullptr);
					
					// test dans le meme boite
					for(l = 0 ; l < Ncand ; l++){
						for(m = l+1 ; m < Ncand ; m++){
							ContactSphSph(cand[l],cand[m],ctl,Nctl);
						}
					}
					// Solo
					if(i+1 < Nx){
						//(i+1)*Ny*Nz+j*Nz+k = num+Ny*Nz
						if((anta = cell[num+Ny*Nz]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(j+1 < Ny){
						//i*Ny*Nz+(j+1)*Nz+k = num+Nz
						if((anta = cell[num+Nz]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(k+1 < Nz){
						//i*Ny*Nz+j*Nz+k+1 = num+1
						if((anta = cell[num+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					// plan XY
					if(i+1 < Nx && j+1 < Ny){
						//(i+1)*Ny*Nz+(j+1)*Nz+k = num + Ny*Nz + Nz = num + Nz*(Ny+1)
						if((anta = cell[num+Nz*(Ny+1)]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i+1 < Nx && j-1 >= 0){
						//(i+1)*Ny*Nz+(j-1)*Nz+k = num + Ny*Nz - Nz = num + Nz*(Ny-1)
						if((anta = cell[num+Nz*(Ny-1)]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					// plan XZ
					if(i+1 < Nx && k+1 < Nz){
						//(i+1)*Ny*Nz+j*Nz+k+1 = num + Ny*Nz + 1
						if((anta = cell[num+Ny*Nz+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i+1 < Nx && k-1 >= 0){
						//(i+1)*Ny*Nz+j*Nz+k-1 = num + Ny*Nz - 1
						if((anta = cell[num+Ny*Nz-1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					// plan YZ
					if(j+1 < Ny && k+1 < Nz){
						//i*Ny*Nz+(j+1)*Nz+k+1 = num + Nz + 1
						if((anta = cell[num+Nz+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(j+1 < Ny && k-1 >= 0){
						//i*Ny*Nz+(j+1)*Nz+k-1 = num + Nz - 1
						if((anta = cell[num+Nz-1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					// tripple
					if(i+1 < Nx && j+1 < Ny && k+1 < Nz){
						//(i+1)*Ny*Nz+(j+1)*Nz+k+1 = num + Nz*(Ny + 1) + 1
						if((anta = cell[num+Nz*(Ny+1)+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i+1 < Nx && j+1 < Ny && k-1 >= 0){
						//(i+1)*Ny*Nz+(j+1)*Nz+k-1 = num + Nz*(Ny + 1) - 1
						if((anta = cell[num+Nz*(Ny+1)-1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i+1 < Nx && j-1 >= 0 && k+1 < Nz){
						//(i+1)*Ny*Nz+(j-1)*Nz+k+1 = num + Nz*(Ny - 1) + 1
						if((anta = cell[num+Nz*(Ny-1)+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
					if(i-1 >= 0 && j+1 < Ny && k+1 < Nz){
						//(i-1)*Ny*Nz+(j+1)*Nz+k+1 = num + Nz*(1-Ny)+1
						if((anta = cell[num+Nz*(1-Ny)+1]) != nullptr){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->TDL()) != nullptr);
						}
					}
				}
			}
		}
	}
}

void ContactDetection::sphPlanContact(const int &Nsph, const int &Npl, int & Nct, std::vector<Sphere> & sph, std::vector<Plan> & pl, Contact *ct, Sphere *cell[], const double rmax) noexcept {
	Sphere *anta;
	int control[Npl];
	
	for(int i = 0 ; i < Npl ; i++){
		if(pl[i].Periodic() == -9)
			control[i] = 1;
		else
			control[i] = 0;
	}
	
	for(int i = 0 ; i < Npl ; i++){
		if(pl[i].Periodic() == -9){
			for(int j = 0 ; j < pl[i].NCell ; j++){
				if((anta = cell[pl[i].Cell[j]]) != nullptr){
					do{
						ContactSphPlan(pl[i], anta, ct, Nct);
					}while((anta = anta->TDL()) != nullptr);
				}
			}
		}
		else{
			if(control[i] == 0){
				Sphere *llistI[Nsph];
				pl[i].ListCount(0);
				for(int j = 0 ; j < pl[i].NCell ; j++){
					if((anta = cell[pl[i].Cell[j]]) != nullptr){
						do{
							ContactSphPlanPeriodic(llistI, pl[i], pl[pl[i].Periodic()], anta, rmax);
						}while((anta = anta->TDL()) != nullptr);
					}
				}
				Sphere *llistK[Nsph];
				int k = pl[i].Periodic();
				pl[k].ListCount(0);
				for(int j = 0 ; j < pl[k].NCell ; j++){
					if((anta = cell[pl[k].Cell[j]]) != nullptr){
						do{
							ContactSphPlanPeriodic(llistK, pl[k], pl[pl[k].Periodic()], anta, rmax);
						}while((anta = anta->TDL()) != nullptr);
					}
				}
				
				for(int j = 0 ; j < pl[i].ListCount() ; j++){
					for(int l = 0 ; l < pl[k].ListCount() ; l++){
						ContactSphSphPeriodic(llistI[j], llistK[l], ct, Nct);
					}
				}
				control[i] = 1;
				control[k] = 1;
			}
		}
	}
}

void ContactDetection::sphPlanContactOMP(const int &Nsph, const int &Npl, int & Nct, int & Ncta, std::vector<Sphere> & sph, std::vector<Plan> & pl, Contact *ct, Contact *cta, Sphere *cell[], const double rmax) noexcept {
	Sphere *anta;
	int control[Npl];
	
	for(int i = 0 ; i < Npl ; i++){
		if(pl[i].Periodic() == -9)
			control[i] = 1;
		else
			control[i] = 0;
	}
	for(int i = 0 ; i < Npl ; i++){
		if(pl[i].Periodic() == -9 && pl[i].GetNgb() == 0){
#pragma omp parallel private(anta)
			{
#pragma omp sections
				{
#pragma omp section
					{
						for(int j = 0 ; j < pl[i].NCell/2 ; j++){
							if((anta = cell[pl[i].Cell[j]]) != nullptr){
								do{
									ContactSphPlan(pl[i], anta, ct, Nct);
								}while((anta = anta->TDL()) != nullptr);
							}
						}
					}
#pragma omp section
					{
						for(int j = pl[i].NCell/2 ; j < pl[i].NCell ; j++){
							if((anta = cell[pl[i].Cell[j]]) != nullptr){
								do{
									ContactSphPlan(pl[i], anta, cta, Ncta);
								}while((anta = anta->TDL()) != nullptr);
							}
						}
					}
				}
			}
		}
		else{
			if(control[i] == 0){
				int k = pl[i].Periodic();
				Sphere *llistK[Nsph];
				Sphere *llistI[Nsph];
#pragma omp parallel private(anta)
				{
#pragma omp sections
					{
#pragma omp section
						{
							pl[i].ListCount(0);
							for(int j = 0 ; j < pl[i].NCell ; j++){
								if((anta = cell[pl[i].Cell[j]]) != nullptr){
									do{
										ContactSphPlanPeriodic(llistI, pl[i],pl[pl[i].Periodic()], anta,rmax);
									}while((anta = anta->TDL()) != nullptr);
								}
							}
						}
#pragma omp section
						{
							pl[k].ListCount(0);
							for(int j = 0 ; j < pl[k].NCell ; j++){
								if((anta = cell[pl[k].Cell[j]]) != nullptr){
									do{
										ContactSphPlanPeriodic(llistK, pl[k],pl[pl[k].Periodic()], anta,rmax);
									}while((anta = anta->TDL()) != nullptr);
								}
							}
						}
					}
				}
				
#pragma omp parallel private(anta)
				{
#pragma omp sections
					{
#pragma omp section
						{
							for(int j = 0 ; j < pl[i].ListCount()/2 ; j++){
								for(int l = 0 ; l < pl[k].ListCount() ; l++){
									ContactSphSphPeriodic(llistI[j],llistK[l], ct, Nct);
								}
							}
						}
						
#pragma omp section
						{
							for(int j = pl[i].ListCount()/2 ; j < pl[i].ListCount() ; j++){
								for(int l = 0 ; l < pl[k].ListCount() ; l++){
									ContactSphSphPeriodic(llistI[j],llistK[l] , cta, Ncta);
								}
							}
						}
					}
				}
				control[i] = 1;
				control[k] = 1;
			}
		}
	}
}

void ContactDetection::sphPlanRContact(const int &Nplr, int & Nct, std::vector<PlanR> & plr, Contact *ct, Sphere *cell[]) noexcept {
	Sphere *anta;
	for(int i = 0 ; i < Nplr ; i++){
		for(int j = 0 ; j < plr[i].NCell ; j++){
			if((anta = cell[plr[i].Cell[j]]) != nullptr){
				do{
					ContactSphPlanR(plr[i], anta, ct, Nct);
				}while((anta = anta->TDL()) != nullptr);
			}
		}
	}
}

void ContactDetection::sphConeContact(const int &Nco, int & Nct, std::vector<Cone> & co, Contact *ct, Sphere *cell[])  noexcept {
	Sphere *anta;
	for(int i = 0 ; i < Nco ; i++){
		for(int j = 0 ; j < co[i].NCell ; j++){
			if((anta = cell[co[i].Cell[j]]) != nullptr){
				do{
					ContactSphCone(co[i], anta, ct, Nct);
				}while((anta = anta->TDL()) != nullptr);
			}
		}
	}
}

void ContactDetection::sphElbowContact(const int &Nelb, int & Nct, std::vector<Elbow> & elb, Contact *ct, Sphere *cell[])  noexcept {
	Sphere *anta;
	for(int i = 0 ; i < Nelb ; i++){
		for(int j = 0 ; j < elb[i].NCell ; j++){
			if((anta = cell[elb[i].Cell[j]]) != nullptr){
				do{
					ContactSphElbow(elb[i], anta, ct, Nct);
				}while((anta = anta->TDL()) != nullptr);
			}
		}
	}
}


void ContactDetection::sphHollowBallContact(const int &Nhb, int & Nct, std::vector<HollowBall> & hb, Contact *ct)  noexcept {
	for(int i = 0 ; i < Nhb ; i++){
		hb[i].ContactDetectionInHollowBall(ct,Nct);
		hb[i].ContactDetectionWithHollowBall(ct,Nct);
	}
}

void ContactDetection::sphContainer(const int & Nsph, const int & Npl, const int & Nplr, const int & Nco, const int & Nelb, const int & Nhb, std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, std::vector<HollowBall> & hb, int & Nct, Contact *ct, Sphere *cell[], const double rmax) noexcept {
	sphPlanContact(Nsph,Npl, Nct, sph, pl, ct, cell,rmax);
	sphPlanRContact(Nplr, Nct, plr, ct, cell);
	sphConeContact(Nco, Nct, co, ct, cell);
	sphElbowContact(Nelb, Nct, elb, ct, cell);
	sphHollowBallContact(Nhb, Nct, hb, ct);
}

void ContactDetection::sphContainerOMP(const int & Nsph, const int & Npl, const int & Nplr, const int & Nco, const int & Nelb, const int & Nhb, std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, std::vector<HollowBall> & hb, int & Nct, Contact *ct, int & Ncta, Contact *cta, Sphere *cell[], const double rmax)  noexcept {
	sphPlanContactOMP(Nsph,Npl, Nct, Ncta, sph, pl, ct, cta, cell,rmax);
	sphPlanRContact(Nplr, Nct, plr, ct, cell);
	sphConeContact(Nco, Nct, co, ct, cell);
	sphElbowContact(Nelb, Nct, elb, ct, cell);
	sphHollowBallContact(Nhb, Nct, hb, ct);
}
