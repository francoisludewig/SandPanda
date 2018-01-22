#include "../../Includes/Contact/ContactDetection.h"
//#include <omp.h>

#include <cmath>

#include "../../Includes/Solids/Velocity.h"
#include "../../Includes/Gravity.h"
#include "../../Includes/Solids/Plan.h"
#include "../../Includes/Solids/PlanR.h"
#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Solids/Elbow.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Data.h"
#include "../../Includes/Solids/HollowBall.h"

int ContactDetection::ContactSphBody(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept {
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
		//P2 = std::fma(px, px, std::fma(py, py, pz*pz));

		if(P2 < Q*Q){
			P2 = std::sqrt(P2);
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

void ContactDetection::ContactSphPlanPeriodic(std::vector<Sphere*>& llist, Plan & p, Plan & p2, Sphere *b, const double rmax) noexcept {
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

void ContactDetection::sphContactAll(std::vector<Sphere> & sph, Contact *ctl, int & Nctl) noexcept {
	int i,j;
	for(i = 0 ; i < sph.size() ; i++){
		for(j = i+1 ; j < sph.size() ; j++){
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

void ContactDetection::sphPlanContact(int & Nct, std::vector<Sphere> & sph, std::vector<Plan> & pl, Contact *ct, Sphere *cell[], const double rmax) noexcept {
	Sphere *anta;
	std::vector<int> control(pl.size(), 0);
	
	for(int i = 0 ; i < pl.size() ; i++){
		if(pl[i].Periodic() == -9)
			control[i] = 1;
		else
			control[i] = 0;
	}
	
	for(int i = 0 ; i < pl.size() ; i++){
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
				//Sphere *llistI[Nsph];
				std::vector<Sphere*> llistI(sph.size(), nullptr);

				pl[i].ListCount(0);
				for(int j = 0 ; j < pl[i].NCell ; j++){
					if((anta = cell[pl[i].Cell[j]]) != nullptr){
						do{
							ContactSphPlanPeriodic(llistI, pl[i], pl[pl[i].Periodic()], anta, rmax);
						}while((anta = anta->TDL()) != nullptr);
					}
				}

				std::vector<Sphere*> llistK(sph.size(), nullptr);
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

void ContactDetection::sphPlanRContact(int & Nct, std::vector<PlanR> & plr, Contact *ct, Sphere *cell[]) noexcept {
	Sphere *anta;
	for(auto& disk: plr) {
		for(int j = 0 ; j < disk.NCell ; j++){
			if((anta = cell[disk.Cell[j]]) != nullptr){
				do{
					ContactSphPlanR(disk, anta, ct, Nct);
				}while((anta = anta->TDL()) != nullptr);
			}
		}
	}
}

void ContactDetection::sphConeContact(int & Nct, std::vector<Cone> & co, Contact *ct, Sphere *cell[])  noexcept {
	Sphere *anta;
	for(auto& cone : co) {
		for(int j = 0 ; j < cone.NCell ; j++){
			if((anta = cell[cone.Cell[j]]) != nullptr){
				do{
					ContactSphCone(cone, anta, ct, Nct);
				}while((anta = anta->TDL()) != nullptr);
			}
		}
	}
}

void ContactDetection::sphElbowContact(int & Nct, std::vector<Elbow> & elb, Contact *ct, Sphere *cell[])  noexcept {
	Sphere *anta;
	for(auto& elbow : elb) {
		for(int j = 0 ; j < elbow.NCell ; j++){
			if((anta = cell[elbow.Cell[j]]) != nullptr){
				do{
					ContactSphElbow(elbow, anta, ct, Nct);
				}while((anta = anta->TDL()) != nullptr);
			}
		}
	}
}


void ContactDetection::sphHollowBallContact(int & Nct, std::vector<HollowBall> & hb, Contact *ct)  noexcept {

	for(auto& hollowBall : hb) {
		hollowBall.ContactDetectionInHollowBall(ct,Nct);
		hollowBall.ContactDetectionWithHollowBall(ct,Nct);
	}
}

void ContactDetection::sphContainer(std::vector<Sphere> & sph, std::vector<Plan> & pl, std::vector<PlanR> & plr, std::vector<Cone> & co, std::vector<Elbow> & elb, std::vector<HollowBall> & hb, int & Nct, Contact *ct, Sphere *cell[], const double rmax) noexcept {
	sphPlanContact(Nct, sph, pl, ct, cell,rmax);
	sphPlanRContact(Nct, plr, ct, cell);
	sphConeContact(Nct, co, ct, cell);
	sphElbowContact(Nct, elb, ct, cell);
	sphHollowBallContact(Nct, hb, ct);
}
