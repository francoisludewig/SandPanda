#include "../../Includes/Contact/ContactDetection.h"
//#include <omp.h>

#include "../../Includes/Object/Velocity.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Object/Plan.h"
#include "../../Includes/Object/PlanR.h"
#include "../../Includes/Object/Cone.h"
#include "../../Includes/Object/Elbow.h"
#include "../../Includes/Object/Sphere.h"
#include "../../Includes/Object/Body.h"
#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Configuration/Data.h"

int ContactDetection::ContactSphBody(Sphere *a, Body *b, Contact *ct, int &Nct) noexcept {
	int Nctfound = 0;
	double Nx,Ny,Nz,xc,yc,zc,norme;
	// Vecteur normal joignant les centres de masse
	Nx = a->x-b->x;
	Ny = a->y-b->y;
	Nz = a->z-b->z;
	norme = sqrt(Nx*Nx+Ny*Ny+Nz*Nz);
	Nx = Nx/norme;
	Ny = Ny/norme;
	Nz = Nz/norme;
	
	// Point central du segment joignant les centres de masse
	xc = a->x - Nx*a->r;
	yc = a->y - Ny*a->r;
	zc = a->z - Nz*a->r;
	
	double delta[b->Ng];
	// Liste des sph de b
	for(int i = b->Ng ; i--;){
		delta[i] = (b->xg[i]-xc)*Nx+(b->yg[i]-yc)*Ny+(b->zg[i]-zc)*Nz;
	}
	
	// Test Contact potentiel
	for(int i = b->Ng ; i--;){
		if(delta[i] > -b->r[i]){
			double px,py,pz,Q,P2;
			px = a->x - b->xg[i];
			py = a->y - b->yg[i];
			pz = a->z - b->zg[i];
			Q = (a->r+b->r[i]);
			P2 = px*px+py*py+pz*pz;
			if(P2 < Q*Q){
				P2 = sqrt(P2);
				ct[Nct].type = 10;
				ct[Nct].delta = P2-(a->r+b->r[i]);
				
				ct[Nct].nx = px/P2;
				ct[Nct].ny = py/P2;
				ct[Nct].nz = pz/P2;
				
				ct[Nct].px = b->xg[i] + px/2.;
				ct[Nct].py = b->yg[i] + py/2.;
				ct[Nct].pz = b->zg[i] + pz/2.;
				
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
	Nx = a->x2-b->x;
	Ny = a->y2-b->y;
	Nz = a->z2-b->z;
	norme = sqrt(Nx*Nx+Ny*Ny+Nz*Nz);
	Nx = Nx/norme;
	Ny = Ny/norme;
	Nz = Nz/norme;
	
	// Point central du segment joignant les centres de masse
	xc = a->x2 - Nx*a->r;
	yc = a->y2 - Ny*a->r;
	zc = a->z2 - Nz*a->r;
	
	double delta[b->Ng];
	// Liste des sph de b
	for(int i = b->Ng ; i--;){
		delta[i] = (b->xg[i]-xc)*Nx+(b->yg[i]-yc)*Ny+(b->zg[i]-zc)*Nz;
	}
	
	// Test Contact potentiel
	for(int i = b->Ng ; i--;){
		if(delta[i] > -b->r[i]){
			double px,py,pz,Q,P2;
			px = a->x2 - b->xg[i];
			py = a->y2 - b->yg[i];
			pz = a->z2 - b->zg[i];
			Q = (a->r+b->r[i]);
			P2 = px*px+py*py+pz*pz;
			if(P2 < Q*Q){
				P2 = sqrt(P2);
				ct[Nct].type = 10;
				ct[Nct].delta = P2-(a->r+b->r[i]);
				
				ct[Nct].nx = px/P2;
				ct[Nct].ny = py/P2;
				ct[Nct].nz = pz/P2;
				
				ct[Nct].px = b->xg[i] + px/2.;
				ct[Nct].py = b->yg[i] + py/2.;
				ct[Nct].pz = b->zg[i] + pz/2.;
				
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
	Nx = a->x-b->x;
	Ny = a->y-b->y;
	Nz = a->z-b->z;
	norme = sqrt(Nx*Nx+Ny*Ny+Nz*Nz);
	Nx = Nx/norme;
	Ny = Ny/norme;
	Nz = Nz/norme;
	
	// Point central du segment joignant les centres de masse
	xc = (ra*a->x+rb*b->x)/(ra+rb);
	yc = (ra*a->y+rb*b->y)/(ra+rb);
	zc = (ra*a->z+rb*b->z)/(ra+rb);
	
	double deltaA[a->Ng];
	double deltaB[b->Ng];
	double dminA = 10,dmaxB = -10;
	double dmaxA = a->Rmax;
	double dminB = -b->Rmax;
	int Na = 0,Nb = 0;
	double Min,Max;
	//int Nct0 = Nct;
	double px,py,pz,Q,P2;
	
	// Liste des sph de a
	for(int i = 0 ; i < a->Ng ; i++){
		deltaA[i] = (a->xg[i]-xc)*Nx+(a->yg[i]-yc)*Ny+(a->zg[i]-zc)*Nz;
		
		if(deltaA[i]-a->r[i] < dminA)dminA = deltaA[i]-a->r[i];
		if(deltaA[i] < dmaxA)Na++;
	}
	// Liste des sph de b
	for(int i = 0 ; i < b->Ng ; i++){
		deltaB[i] = (b->xg[i]-xc)*Nx+(b->yg[i]-yc)*Ny+(b->zg[i]-zc)*Nz;
		if(deltaB[i]+b->r[i] > dmaxB)dmaxB = deltaB[i]+b->r[i];
		if(deltaB[i] > dminB)Nb++;
	}
	
	
	// Tolerence de securite
	dminA -= a->Rmax;
	dminB -= b->Rmax;
	dmaxA += a->Rmax;
	dmaxB += b->Rmax;
	
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
	
	for(int i = 0 ; i < a->Ng ; i++){
		for(int j = 0 ; j < b->Ng ; j++){
			Ncontrol++;
			if(deltaA[i] < Max && deltaB[j] < Max && deltaA[i] > Min && deltaB[j] > Min){
				Ntest++;
				//double px,py,pz,Q,P2;
				px = a->xg[i] - b->xg[j];
				py = a->yg[i] - b->yg[j];
				pz = a->zg[i] - b->zg[j];
				Q = (a->r[i]+b->r[j]);
				P2 = px*px+py*py+pz*pz;
				if(P2 < Q*Q){
					
					P2 = sqrt(P2);
					ct[Nct].type = 20;
					ct[Nct].delta = P2-(a->r[i]+b->r[j]);
					
					ct[Nct].nx = px/P2;
					ct[Nct].ny = py/P2;
					ct[Nct].nz = pz/P2;
					
					ct[Nct].px = b->xg[j] + px/2.;
					ct[Nct].py = b->yg[j] + py/2.;
					ct[Nct].pz = b->zg[j] + pz/2.;
					
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
	if(a->bodies == -9 && b->bodies == -9){
		double px,py,pz,Q,P2;
		px = a->x - b->x;
		py = a->y - b->y;
		pz = a->z - b->z;
		Q = (a->r+b->r);
		P2 = px*px+py*py+pz*pz;
		if(P2 < Q*Q){
			P2 = sqrt(P2);
			ct[Nct].type = 0;
			ct[Nct].delta = P2-Q;
			ct[Nct].nx = px/P2;
			ct[Nct].ny = py/P2;
			ct[Nct].nz = pz/P2;
			
			ct[Nct].px = b->x + px/2.;
			ct[Nct].py = b->y + py/2.;
			ct[Nct].pz = b->z + pz/2.;
			
			ct[Nct].sa = a;
			ct[Nct].sb = b;
			Nct++;
			rtn = 1;
		}
	}
	if(a->bodies != -9 && b->bodies == -9){
		rtn = ContactSphBody(b, a->b,ct,Nct);
	}
	if(a->bodies == -9 && b->bodies != -9){
		rtn = ContactSphBody(a, b->b,ct,Nct);
	}
	if(a->bodies != -9 && b->bodies != -9){
		 rtn = ContactBodyBody(a->b, b->b,ct,Nct,a->r,b->r);
	}
	return rtn;
}

void ContactDetection::ContactSphSphPeriodic(Sphere *a, Sphere *b, Contact *ct, int & Nct) noexcept {
	int rtn = 0;
	if(a->Num() != b->Num()){
		if((a->bodies == -9 && b->bodies == -9)){
			double px,py,pz,P2,Q;
			px = a->x2 - b->x;
			py = a->y2 - b->y;
			pz = a->z2 - b->z;
			P2 = px*px+py*py+pz*pz;
			Q = (a->r+b->r);
			if(P2 < Q*Q){
				P2 = sqrt(P2);
				ct[Nct].type = 0;
				ct[Nct].delta = P2-(a->r+b->r);
				ct[Nct].nx = px/P2;
				ct[Nct].ny = py/P2;
				ct[Nct].nz = pz/P2;
				ct[Nct].px = b->x + px/2;
				ct[Nct].py = b->y + py/2;
				ct[Nct].pz = b->z + pz/2;
				ct[Nct].sa = a;
				ct[Nct].sb = b;
				Nct++;
			}
		}
		
		else{
			if(a->bodies != -9 && b->bodies == -9){
				rtn = ContactSphBodyPeriodic(b, a->b,ct,Nct);
			}
			if(a->bodies == -9 && b->bodies != -9){
				rtn = ContactSphBodyPeriodic(a, b->b,ct,Nct);
			}
			if(a->bodies != -9 && b->bodies != -9){
				double tpx,tpy,tpz;
				tpx = a->x;
				tpy = a->y;
				tpz = a->z;
				(a->b)->x = a->x2;
				(a->b)->y = a->y2;
				(a->b)->z = a->z2;
				(a->b)->UpDateLinkedSphereTp();
				
				rtn = ContactBodyBody(a->b, b->b,ct,Nct,a->r,b->r);
				(a->b)->x = tpx;
				(a->b)->y = tpy;
				(a->b)->z = tpz;
				(a->b)->UpDateLinkedSphereTp();
				
			}
		}
	}
}

void ContactDetection::ContactBodyPlan(Plan & p, Body *b, Contact *ct, int & Nct) noexcept {
	double px,py,pz,pn,pt,ps,nn,nnx,nny,nnz;
	for(int i = 0 ; i < b->Ng ; i++){
		px = b->xg[i] - b->r[i]*p.nx;
		py = b->yg[i] - b->r[i]*p.ny;
		pz = b->zg[i] - b->r[i]*p.nz;
		pn = (px-p.x)*p.nx+(py-p.y)*p.ny+(pz-p.z)*p.nz;
		pt = (px-p.x)*p.tx+(py-p.y)*p.ty+(pz-p.z)*p.tz;
		ps = (px-p.x)*p.sx+(py-p.y)*p.sy+(pz-p.z)*p.sz;
		if((fabs(pt) < p.dt) && (fabs(ps) < p.ds)){
			if( (pn <= 0.0) && (pn > -b->r[i])){
				ct[Nct].type = 11;
				ct[Nct].delta = pn;
				ct[Nct].px = px;
				ct[Nct].py = py;
				ct[Nct].pz = pz;
				ct[Nct].nx = -p.nx;
				ct[Nct].ny = -p.ny;
				ct[Nct].nz = -p.nz;
				ct[Nct].pa = &p;
				ct[Nct].ba = b;
				ct[Nct].nba = i;
				Nct++;
			}
			if(p.inAndOut == 1){
				px = b->xg[i] + b->r[i]*p.nx;
				py = b->yg[i] + b->r[i]*p.ny;
				pz = b->zg[i] + b->r[i]*p.nz;
				pn = (px-p.x)*(-p.nx)+(py-p.y)*(-p.ny)+(pz-p.z)*(-p.nz);
				pt = (px-p.x)*p.tx+(py-p.y)*p.ty+(pz-p.z)*p.tz;
				ps = (px-p.x)*p.sx+(py-p.y)*p.sy+(pz-p.z)*p.sz;
				if( (pn <= 0.0) && (pn > -b->r[i])){
					ct[Nct].type = 11;
					ct[Nct].delta = pn;
					ct[Nct].px = px;
					ct[Nct].py = py;
					ct[Nct].pz = pz;
					ct[Nct].nx = p.nx;
					ct[Nct].ny = p.ny;
					ct[Nct].nz = p.nz;
					ct[Nct].pa = &p;
					ct[Nct].ba = b;
					ct[Nct].nba = i;
					Nct++;
				}
			}
		}
		else{
			// pt ok seul
			if((fabs(pt) < p.dt/2) && (fabs(ps) > p.ds/2)){
				ps = ps/fabs(ps)*p.ds/2;
			}
			// ps ok seul
			if((fabs(pt) > p.dt/2) && (fabs(ps) < p.ds/2)){
				pt = pt/fabs(pt)*p.dt/2;
			}
			// aucun ok
			if((fabs(pt) > p.dt/2) && (fabs(ps) > p.ds/2)){
				ps = ps/fabs(ps)*p.ds/2;
				pt = pt/fabs(pt)*p.dt/2;
			}
			// Point du plan candidat au contact
			px = p.x + ps*p.sx + pt*p.tx;
			py = p.y + ps*p.sy + pt*p.ty;
			pz = p.z + ps*p.sz + pt*p.tz;
			
			nnx = px-b->x;
			nny = py-b->y;
			nnz = pz-b->z;
			nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
			if(nn <= b->r[i]){
				ct[Nct].delta = nn - b->r[i];
				ct[Nct].type = 11;
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
	if(b->bodies == -9){
		double px,py,pz,pn,pt,ps,nn,nnx,nny,nnz;
		px = b->x - b->r*p.nx;
		py = b->y - b->r*p.ny;
		pz = b->z - b->r*p.nz;
		pn = (px-p.x)*p.nx+(py-p.y)*p.ny+(pz-p.z)*p.nz;
		pt = (px-p.x)*p.tx+(py-p.y)*p.ty+(pz-p.z)*p.tz;
		ps = (px-p.x)*p.sx+(py-p.y)*p.sy+(pz-p.z)*p.sz;
		if((fabs(pt) < p.dt/2) && (fabs(ps) < p.ds/2)){
			if( (pn <= 0.0) && (pn > -b->r)){
				ct[Nct].type = 1;
				ct[Nct].delta = pn;
				ct[Nct].px = px;
				ct[Nct].py = py;
				ct[Nct].pz = pz;
				ct[Nct].nx = -p.nx;
				ct[Nct].ny = -p.ny;
				ct[Nct].nz = -p.nz;
				//printf("n = (%e,%e,%e)\n", ct[Nct].nx, ct[Nct].ny, ct[Nct].nz);
				p.Normal(&ct[Nct],b);
				//if(p.sigma != 0)
				//	printf("n = (%e,%e,%e)\n\n", ct[Nct].nx, ct[Nct].ny, ct[Nct].nz);
				
				ct[Nct].pa = &p;
				ct[Nct].sa = b;
				Nct++;
			}
			else{
				if(p.sigma != 0)
					b->ct_pl = 0;
			}
			if(p.inAndOut == 1){
				px = b->x + b->r*p.nx;
				py = b->y + b->r*p.ny;
				pz = b->z + b->r*p.nz;
				pn = (px-p.x)*(-p.nx)+(py-p.y)*(-p.ny)+(pz-p.z)*(-p.nz);
				pt = (px-p.x)*p.tx+(py-p.y)*p.ty+(pz-p.z)*p.tz;
				ps = (px-p.x)*p.sx+(py-p.y)*p.sy+(pz-p.z)*p.sz;
				if( (pn <= 0.0) && (pn > -b->r)){
					ct[Nct].type = 1;
					ct[Nct].delta = pn;
					ct[Nct].px = px;
					ct[Nct].py = py;
					ct[Nct].pz = pz;
					ct[Nct].nx = p.nx;
					ct[Nct].ny = p.ny;
					ct[Nct].nz = p.nz;
					p.Normal(&ct[Nct],b);
					ct[Nct].pa = &p;
					ct[Nct].sa = b;
					Nct++;
				}
				else{
					if(p.sigma != 0)
						b->ct_pl = 0;
				}
			}
		}
		else{
			// pt ok seul
			if((fabs(pt) < p.dt/2) && (fabs(ps) > p.ds/2)){
				ps = ps/fabs(ps)*p.ds/2;
			}
			// ps ok seul
			if((fabs(pt) > p.dt/2) && (fabs(ps) < p.ds/2)){
				pt = pt/fabs(pt)*p.dt/2;
			}
			// aucun ok
			if((fabs(pt) > p.dt/2) && (fabs(ps) > p.ds/2)){
				ps = ps/fabs(ps)*p.ds/2;
				pt = pt/fabs(pt)*p.dt/2;
			}
			// Point du plan candidat au contact
			px = p.x + ps*p.sx + pt*p.tx;
			py = p.y + ps*p.sy + pt*p.ty;
			pz = p.z + ps*p.sz + pt*p.tz;
			
			nnx = px-b->x;
			nny = py-b->y;
			nnz = pz-b->z;
			nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
			if(nn <= b->r){
				ct[Nct].delta = nn - b->r;
				ct[Nct].type = 1;
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
		ContactBodyPlan(p,b->b,ct,Nct);
}

void ContactDetection::ContactSphPlanPeriodic(Sphere *llist[], Plan & p, Plan & p2, Sphere *b, const double rmax) noexcept {
	double px,py,pz,pn,pt,ps;
	px = b->x;
	py = b->y;
	pz = b->z;
	pn = (px-p.x)*p.nx+(py-p.y)*p.ny+(pz-p.z)*p.nz;
	pt = (px-p.x)*p.tx+(py-p.y)*p.ty+(pz-p.z)*p.tz;
	ps = (px-p.x)*p.sx+(py-p.y)*p.sy+(pz-p.z)*p.sz;
	if( (pn <= (b->r+rmax)) && (pn > -2*b->r) && (fabs(pt) < p.dt/2+b->r) &&
	   (fabs(ps) < p.ds/2+b->r) ){
		pt = (px-p2.x)*p2.tx+(py-p2.y)*p2.ty+(pz-p2.z)*p2.tz;
		ps = (px-p2.x)*p2.sx+(py-p2.y)*p2.sy+(pz-p2.z)*p2.sz;
		b->x2 = p2.x + (-pn)*p2.nx + pt*p2.tx + ps*p2.sx;
		b->y2 = p2.y + (-pn)*p2.ny + pt*p2.ty + ps*p2.sy;
		b->z2 = p2.z + (-pn)*p2.nz + pt*p2.tz + ps*p2.sz;
		llist[p.Nlist] = b;
		p.list[p.Nlist] = b->num;
		p.Nlist++;
	}
}

void ContactDetection::ContactBodyPlanR(PlanR & p, Body *b, Contact *ct, int & Nct) noexcept {
	double px,py,pz,pn,pt,ps;
	for(int i = 0 ; i < b->Ng ; i++){
		px = b->xg[i] - b->r[i]*p.nx;
		py = b->yg[i] - b->r[i]*p.ny;
		pz = b->zg[i] - b->r[i]*p.nz;
		pn = (px-p.x)*p.nx+(py-p.y)*p.ny+(pz-p.z)*p.nz;
		pt = (px-p.x)*p.tx+(py-p.y)*p.ty+(pz-p.z)*p.tz;
		ps = (px-p.x)*p.sx+(py-p.y)*p.sy+(pz-p.z)*p.sz;
		if((pn >= -b->r[i]) && (pn < 0.0) && (sqrt(pt*pt+ps*ps) < p.r)){
			ct[Nct].delta = pn;
			ct[Nct].type = 12;
			ct[Nct].px = px;
			ct[Nct].py = py;
			ct[Nct].pz = pz;
			ct[Nct].nx = -p.nx;
			ct[Nct].ny = -p.ny;
			ct[Nct].nz = -p.nz;
			ct[Nct].ba = b;
			ct[Nct].nba = i;
			ct[Nct].par = &p;
			Nct++;
		}
	}
}

void ContactDetection::ContactSphPlanR(PlanR & p, Sphere *b, Contact *ct, int & Nct) noexcept {
	if(b->bodies == -9){
		double px,py,pz,pn,pt,ps;
		px = b->x - b->r*p.nx;
		py = b->y - b->r*p.ny;
		pz = b->z - b->r*p.nz;
		pn = (px-p.x)*p.nx+(py-p.y)*p.ny+(pz-p.z)*p.nz;
		pt = (px-p.x)*p.tx+(py-p.y)*p.ty+(pz-p.z)*p.tz;
		ps = (px-p.x)*p.sx+(py-p.y)*p.sy+(pz-p.z)*p.sz;
		if((pn >= -b->r) && (pn < 0.0) && (sqrt(pt*pt+ps*ps) < p.r)){
			ct[Nct].delta = pn;
			ct[Nct].type = 2;
			ct[Nct].px = px;
			ct[Nct].py = py;
			ct[Nct].pz = pz;
			ct[Nct].nx = -p.nx;
			ct[Nct].ny = -p.ny;
			ct[Nct].nz = -p.nz;
			ct[Nct].sa = b;
			ct[Nct].par = &p;
			Nct++;
		}
	}
	else
		ContactBodyPlanR(p,b->b,ct,Nct);
}

void ContactDetection::ContactBodyCone(Cone & p, Body *b, Contact *ct, int & Nct) noexcept {
	double px,py,pz,pn,pt,ps,N,Tx,Ty,Tz,Nx,Ny,Nz,delta2,Qn,Qt,dr,h;
	double nnx,nny,nnz,nn;
	double a,bb,Y;
	for(int i = 0 ; i < b->Ng ; i++){
		px = b->xg[i];
		py = b->yg[i];
		pz = b->zg[i];
		pn = (px-p.x)*p.nx+(py-p.y)*p.ny+(pz-p.z)*p.nz;
		pt = (px-p.x)*p.tx+(py-p.y)*p.ty+(pz-p.z)*p.tz;
		ps = (px-p.x)*p.sx+(py-p.y)*p.sy+(pz-p.z)*p.sz;
		
		if(fabs(pn) <= p.h/2 + b->r[i]){
			N = sqrt(pt*pt+ps*ps);
			Tx = (pt*p.tx + ps*p.sx)/N;
			Ty = (pt*p.ty + ps*p.sy)/N;
			Tz = (pt*p.tz + ps*p.sz)/N;
			Nx = p.nx;
			Ny = p.ny;
			Nz = p.nz;
			dr = p.r0 -  p.r1;
			h = p.h;
			if(dr == 0){
				if(fabs(pn) <= p.h/2){
					if(fabs(N-p.r0) <= b->r[i]){
						px = p.x + pn*Nx + p.r0*Tx;
						py = p.y + pn*Ny + p.r0*Ty;
						pz = p.z + pn*Nz + p.r0*Tz;
						ct[Nct].delta = fabs(N-p.r0) - b->r[i];
						ct[Nct].type = 13;
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
					pn = pn/fabs(pn)*p.h/2.;
					px = p.x + pn*Nx + p.r0*Tx;
					py = p.y + pn*Ny + p.r0*Ty;
					pz = p.z + pn*Nz + p.r0*Tz;
					nnx = px-b->xg[i];
					nny = py-b->yg[i];
					nnz = pz-b->zg[i];
					nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
					if(nn < b->r[i]){
						ct[Nct].delta = nn - b->r[i];
						ct[Nct].type = 13;
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
				bb = h*(p.r0/dr-0.5);
				Y = sqrt(dr*dr+h*h);
				delta2 = -(N - pn/a + bb/a)*h/Y;
				if(fabs(delta2) <= b->r[i]){
					Qn = pn + delta2*dr/Y;
					Qt = N + delta2*h/Y;
					px = p.x + Qn*Nx + Qt*Tx;
					py = p.y + Qn*Ny + Qt*Ty;
					pz = p.z + Qn*Nz + Qt*Tz;
					if(Qn < h/2. && Qn > -h/2.){
						ct[Nct].delta = fabs(delta2) - b->r[i];
						ct[Nct].type = 13;
						ct[Nct].px = px;
						ct[Nct].py = py;
						ct[Nct].pz = pz;
						ct[Nct].nx = (px-b->xg[i])/fabs(delta2);
						ct[Nct].ny = (py-b->yg[i])/fabs(delta2);
						ct[Nct].nz = (pz-b->zg[i])/fabs(delta2);
						ct[Nct].ba = b;
						ct[Nct].nba = i;
						ct[Nct].cn = &p;
						Nct++;
					}
					else{
						if(Qn >= h/2.){
							Qn = h/2.;
							Qt = p.r1;
						}
						else{
							Qn = -h/2.;
							Qt = p.r0;
						}
						px = p.x + Qn*Nx + Qt*Tx;
						py = p.y + Qn*Ny + Qt*Ty;
						pz = p.z + Qn*Nz + Qt*Tz;
						nnx = px-b->xg[i];
						nny = py-b->yg[i];
						nnz = pz-b->zg[i];
						nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
						if(nn <= b->r[i]){
							ct[Nct].delta = nn - b->r[i];
							ct[Nct].type = 13;
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
	if(b->bodies == -9){
		double px,py,pz,pn,pt,ps,N,Tx,Ty,Tz,Nx,Ny,Nz,delta2,Qn,Qt,dr,h;
		double nnx,nny,nnz,nn;
		double a,bb,Y;
		px = b->x;
		py = b->y;
		pz = b->z;
		pn = (px-p.x)*p.nx+(py-p.y)*p.ny+(pz-p.z)*p.nz;
		pt = (px-p.x)*p.tx+(py-p.y)*p.ty+(pz-p.z)*p.tz;
		ps = (px-p.x)*p.sx+(py-p.y)*p.sy+(pz-p.z)*p.sz;
		
		if(fabs(pn) <= p.h/2 + b->r){
			N = sqrt(pt*pt+ps*ps);
			Tx = (pt*p.tx + ps*p.sx)/N;
			Ty = (pt*p.ty + ps*p.sy)/N;
			Tz = (pt*p.tz + ps*p.sz)/N;
			Nx = p.nx;
			Ny = p.ny;
			Nz = p.nz;
			dr = p.r0 -  p.r1;
			h = p.h;
			
			if(dr == 0){
				if(fabs(pn) <= p.h/2){
					if(fabs(N-p.r0) <= b->r){
						px = p.x + pn*Nx + p.r0*Tx;
						py = p.y + pn*Ny + p.r0*Ty;
						pz = p.z + pn*Nz + p.r0*Tz;
						ct[Nct].delta = fabs(N-p.r0) - b->r;
						ct[Nct].type = 3;
						ct[Nct].px = px;
						ct[Nct].py = py;
						ct[Nct].pz = pz;
						if(p.in == 0){
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
					pn = pn/fabs(pn)*p.h/2.;
					px = p.x + pn*Nx + p.r0*Tx;
					py = p.y + pn*Ny + p.r0*Ty;
					pz = p.z + pn*Nz + p.r0*Tz;
					nnx = px-b->x;
					nny = py-b->y;
					nnz = pz-b->z;
					nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
					if(nn < b->r){
						ct[Nct].delta = nn - b->r;
						ct[Nct].type = 3;
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
				bb = h*(p.r0/dr-0.5);
				Y = sqrt(dr*dr+h*h);
				delta2 = -(N - pn/a + bb/a)*h/Y;
				
				if(fabs(delta2) <= b->r){
					Qn = pn + delta2*dr/Y;
					Qt = N + delta2*h/Y;
					px = p.x + Qn*Nx + Qt*Tx;
					py = p.y + Qn*Ny + Qt*Ty;
					pz = p.z + Qn*Nz + Qt*Tz;
					if(Qn < h/2. && Qn > -h/2.){
						ct[Nct].delta = fabs(delta2) - b->r;
						ct[Nct].type = 3;
						ct[Nct].px = px;
						ct[Nct].py = py;
						ct[Nct].pz = pz;
						nnx = (px-b->x);
						nny = (py-b->y);
						nnz = (pz-b->z);
						ct[Nct].nx = (px-b->x)/fabs(delta2);
						ct[Nct].ny = (py-b->y)/fabs(delta2);
						ct[Nct].nz = (pz-b->z)/fabs(delta2);
						ct[Nct].sa = b;
						ct[Nct].cn = &p;
						Nct++;
					}
					else{
						if(Qn >= h/2.){
							Qn = h/2.;
							Qt = p.r1;
						}
						else{
							Qn = -h/2.;
							Qt = p.r0;
						}
						px = p.x + Qn*Nx + Qt*Tx;
						py = p.y + Qn*Ny + Qt*Ty;
						pz = p.z + Qn*Nz + Qt*Tz;
						nnx = px-b->x;
						nny = py-b->y;
						nnz = pz-b->z;
						nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
						if(nn <= b->r){
							ct[Nct].delta = nn - b->r;
							ct[Nct].type = 3;
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
		ContactBodyCone(p,b->b,ct,Nct);
}

void ContactDetection::ContactBodyElbow(Elbow & p, Body *b, Contact *ct, int & Nct) noexcept {
	double rl,D,pn,pt,ps,alphal;
	double cx,cy,cz,nx,ny,nz;
	for(int i = 0 ; i < b->Ng ; i++){
		pn = (b->xg[i]-p.x)*p.nx + (b->yg[i]-p.y)*p.ny + (b->zg[i]-p.z)*p.nz;
		pt = (b->xg[i]-p.x)*p.tx + (b->yg[i]-p.y)*p.ty + (b->zg[i]-p.z)*p.tz;
		ps = (b->xg[i]-p.x)*p.sx + (b->yg[i]-p.y)*p.sy + (b->zg[i]-p.z)*p.sz;
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
				D = sqrt(pow(b->x-cx,2)+pow(b->y-cy,2)+pow(b->z-cz,2));
				if(p.r-(D+b->r[i]) < 0){
					ct[Nct].delta =  p.r-(D+b->r[i]);
					ct[Nct].type = 14;
					nx = -(cx - b->xg[i]);
					ny = -(cy - b->yg[i]);
					nz = -(cz - b->zg[i]);
					D = sqrt(nx*nx+ny*ny+nz*nz);
					ct[Nct].nx = nx/D;
					ct[Nct].ny = ny/D;
					ct[Nct].nz = nz/D;
					ct[Nct].px = b->xg[i] + b->r[i]*ct[Nct].nx;
					ct[Nct].py = b->yg[i] + b->r[i]*ct[Nct].ny;
					ct[Nct].pz = b->zg[i] + b->r[i]*ct[Nct].nz;
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
	if(b->bodies == -9){
		double rl,D,pn,pt,ps,alphal;
		double cx,cy,cz,nx,ny,nz;
		
		pn = (b->x-p.x)*p.nx + (b->y-p.y)*p.ny + (b->z-p.z)*p.nz;
		pt = (b->x-p.x)*p.tx + (b->y-p.y)*p.ty + (b->z-p.z)*p.tz;
		ps = (b->x-p.x)*p.sx + (b->y-p.y)*p.sy + (b->z-p.z)*p.sz;
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
				D = sqrt(pow(b->x-cx,2)+pow(b->y-cy,2)+pow(b->z-cz,2));
				if(p.r-(D+b->r) < 0){
					ct[Nct].delta =  p.r-(D+b->r);
					ct[Nct].type = 4;
					nx = -(cx - b->x);
					ny = -(cy - b->y);
					nz = -(cz - b->z);
					D = sqrt(nx*nx+ny*ny+nz*nz);
					ct[Nct].nx = nx/D;
					ct[Nct].ny = ny/D;
					ct[Nct].nz = nz/D;
					ct[Nct].px = b->x + b->r*ct[Nct].nx;
					ct[Nct].py = b->y + b->r*ct[Nct].ny;
					ct[Nct].pz = b->z + b->r*ct[Nct].nz;
					ct[Nct].sa = b;
					ct[Nct].ew = &p;
					Nct++;
				}
			}
		}
	}
	else
		ContactBodyElbow(p,b->b,ct,Nct);
}

void ContactDetection::linkedCell(vector<Sphere> & sph, const int Nsph, const Data *dat, Sphere *cell[]) noexcept {
	int x,y,z;
	// Initialisation du tableau tdl
	for(int i = Nsph ; i--;)
		sph[i].tdl = NULL;
	
	// Initialisation du tableau Cell
	for(int i = dat->Ncellmax ; i--;)
		cell[i] = NULL;
	
	// Classement des grains dans les cellules
	for(int i = Nsph ; i--;){
		if(sph[i].HollowballNum() == -9){
			x = (int)((sph[i].x - dat->xmin)/dat->ax);
			y = (int)((sph[i].y - dat->ymin)/dat->ay);
			z = (int)((sph[i].z - dat->zmin)/dat->az);
			if(x < dat->Nx && y < dat->Ny && z < dat->Nz && x >= 0 && y >= 0 && z >= 0){
				//printf("Found it\n");
				//printf("%d in %d\n",i,x*dat->Ny*dat->Nz+y*dat->Nz+z);
				sph[i].tdl = cell[x*dat->Ny*dat->Nz+y*dat->Nz+z];
				cell[x*dat->Ny*dat->Nz+y*dat->Nz+z] = &sph[i];
			}
		}
	}
}

void ContactDetection::listCellForPlan(Data *dat, vector<Plan> & pl, int & Npl, Gravity& gt) noexcept {
	int a,b;
	int i,j,k;
	int I,num;
	double x,y,z,pn,pt,ps;
	double dn,dt,ds;
	double ox,oy,oz;
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	
	double Sox,Soy,Soz;
	double Sq0,Sq1,Sq2,Sq3;
	
	int doublon;
	int *localCell;
	int localNcell;
	
	double Vmax,time,dtime,dist;
	int Nts;
	localCell =(int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
	
	for(a = 0 ; a < Npl ; a++){
		if(pl[a].Force == 0){
			Vmax = pl[a].Vmax();
			if(Vmax < pl[a].Wmax()*pl[a].dt)Vmax = pl[a].Wmax()*pl[a].dt;
			if(Vmax < pl[a].Wmax()*pl[a].ds)Vmax = pl[a].Wmax()*pl[a].ds;
			
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
			
			Sox = pl[a].x;
			Soy = pl[a].y;
			Soz = pl[a].z;
			
			Sq0 = pl[a].q0;
			Sq1 = pl[a].q1;
			Sq2 = pl[a].q2;
			Sq3 = pl[a].q3;
			printf("Nts(%d) = %d\n",a,Nts);
			for(b = 0 ; b <= Nts ; b++){
				if(b != 0){
					pl[a].UpDateVelocity(b*dtime,dtime,gt);
					pl[a].Move(dtime);
				}
				ox = pl[a].x;
				oy = pl[a].y;
				oz = pl[a].z;
				nx = pl[a].nx;
				ny = pl[a].ny;
				nz = pl[a].nz;
				tx = pl[a].tx;
				ty = pl[a].ty;
				tz = pl[a].tz;
				sx = pl[a].sx;
				sy = pl[a].sy;
				sz = pl[a].sz;
				
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
							if( (fabs(pt) < (pl[a].dt/2+dt)) &&
							   (fabs(ps) < (pl[a].ds/2+ds)) &&
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
			
			pl[a].x = Sox;
			pl[a].y = Soy;
			pl[a].z = Soz;
			pl[a].q0 = Sq0;
			pl[a].q1 = Sq1;
			pl[a].q2 = Sq2;
			pl[a].q3 = Sq3;
			pl[a].ComputeBase();
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

void ContactDetection::listCellForPlanR(Data *dat, vector<PlanR> & plr, int & Nplr, Gravity& gt) noexcept {
	int a,b;
	int i,j,k;
	int I,num;
	double x,y,z,pn,pt,ps;
	double dn;
	double ox,oy,oz;
	double nx,ny,nz;
	double tx,ty,tz;
	double sx,sy,sz;
	
	double Sox,Soy,Soz;
	double Sq0,Sq1,Sq2,Sq3;
	int doublon;
	int *localCell;
	int localNcell;
	
	double Vmax,time,dtime,dist;
	int Nts;
	
	localCell =(int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
	
	for(a = 0 ;  a < Nplr ; a++){
		
		if(plr[a].Force == 0){
			
			Vmax = plr[a].Vmax();
			
			if(Vmax < plr[a].Wmax()*plr[a].r)Vmax = plr[a].Wmax()*plr[a].r;
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
			
			Sox = plr[a].x;
			Soy = plr[a].y;
			Soz = plr[a].z;
			
			Sq0 = plr[a].q0;
			Sq1 = plr[a].q1;
			Sq2 = plr[a].q2;
			Sq3 = plr[a].q3;
		
			for(b = 0 ; b <= Nts ; b++){
				if(b != 0){
					plr[a].UpDateVelocity(b*dtime,dtime,gt);
					plr[a].Move(dtime);
				}
				ox = plr[a].x;
				oy = plr[a].y;
				oz = plr[a].z;
				nx = plr[a].nx;
				ny = plr[a].ny;
				nz = plr[a].nz;
				tx = plr[a].tx;
				ty = plr[a].ty;
				tz = plr[a].tz;
				sx = plr[a].sx;
				sy = plr[a].sy;
				sz = plr[a].sz;
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
							if( (sqrt(pt*pt+ps*ps) < plr[a].r+2*dn) &&
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
			
			plr[a].x = Sox;
			plr[a].y = Soy;
			plr[a].z = Soz;
			
			plr[a].q0 = Sq0;
			plr[a].q1 = Sq1;
			plr[a].q2 = Sq2;
			plr[a].q3 = Sq3;
			plr[a].ComputeBase();
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

void ContactDetection::listCellForCone(Data *dat, vector<Cone> & co, int & Nco, Gravity& gt) noexcept {
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
	
	
	double Sox,Soy,Soz;
  double Sq0,Sq1,Sq2,Sq3;
	
	double Vmax,time,dtime,dist;
	int Nts,b2;
	
	
	localCell =(int*)malloc(dat->Nx*dat->Ny*dat->Nz*sizeof(int));
	
	for(numCone = 0 ;  numCone < Nco ; numCone++){
		if(co[numCone].Force == 0){
			Vmax = co[numCone].Vmax();
			if(Vmax < co[numCone].Wmax()*co[numCone].h/2.)Vmax = co[numCone].Wmax()*co[numCone].h/2.;
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
			
			Sox = co[numCone].x;
			Soy = co[numCone].y;
			Soz = co[numCone].z;
			
			Sq0 = co[numCone].q0;
			Sq1 = co[numCone].q1;
			Sq2 = co[numCone].q2;
			Sq3 = co[numCone].q3;
			printf("Nts(%d) = %d\n",numCone,Nts);
			
			for(b2 = 0 ; b2 <= Nts ; b2++){
				if(b2 != 0){
					co[numCone].UpDateVelocity(b2*dtime,dtime,gt);
					co[numCone].Move(dtime);
				}
				ox = co[numCone].x;
				oy = co[numCone].y;
				oz = co[numCone].z;
				nx = co[numCone].nx;
				ny = co[numCone].ny;
				nz = co[numCone].nz;
				tx = co[numCone].tx;
				ty = co[numCone].ty;
				tz = co[numCone].tz;
				sx = co[numCone].sx;
				sy = co[numCone].sy;
				sz = co[numCone].sz;
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
							
							if(fabs(pn) <= co[numCone].h/2+dn){
								N = sqrt(pt*pt+ps*ps);
								if(ps >= 0)
									beta = acos(pt/N);
								else
									beta = 2*M_PI-acos(pt/N);
								
								// 3D -> 2D
								dr = co[numCone].r0 -  co[numCone].r1;
								h = co[numCone].h;
								if(dr == 0){
									if(fabs(N-co[numCone].r0) <= dn*2){
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
									b = h*(co[numCone].r0/dr-0.5);
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
			
			co[numCone].x = Sox;
			co[numCone].y = Soy;
			co[numCone].z = Soz;
			co[numCone].q0 = Sq0;
			co[numCone].q1 = Sq1;
			co[numCone].q2 = Sq2;
			co[numCone].q3 = Sq3;
			co[numCone].ComputeBase();
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

void ContactDetection::listCellForElbow(Data *dat, vector<Elbow> & el, int & Nelb) noexcept {
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

void ContactDetection::sphContactAll(const int & Nsph, vector<Sphere> & sph, Contact *ctl, int & Nctl) noexcept {
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
				
				if((cand[Ncand] = cell[num]) != NULL){
					//printf("(%d,%d,%d) => %d\n",i,j,k,num);
					do{
						//cand[Ncand]->affiche();
						Ncand++;
					}while((cand[Ncand] = cand[Ncand-1]->tdl) != NULL);
					
					// test dans le meme boite
					for(l = 0 ; l < Ncand ; l++){
						for(m = l+1 ; m < Ncand ; m++){
							ContactSphSph(cand[l],cand[m],ctl,Nctl);
						}
					}
					// Solo
					if(i+1 < Nx){
						//(i+1)*Ny*Nz+j*Nz+k = num+Ny*Nz
						if((anta = cell[num+Ny*Nz]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					if(j+1 < Ny){
						//i*Ny*Nz+(j+1)*Nz+k = num+Nz
						if((anta = cell[num+Nz]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					if(k+1 < Nz){
						//i*Ny*Nz+j*Nz+k+1 = num+1
						if((anta = cell[num+1]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					// plan XY
					if(i+1 < Nx && j+1 < Ny){
						//(i+1)*Ny*Nz+(j+1)*Nz+k = num + Ny*Nz + Nz = num + Nz*(Ny+1)
						if((anta = cell[num+Nz*(Ny+1)]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					if(i+1 < Nx && j-1 >= 0){
						//(i+1)*Ny*Nz+(j-1)*Nz+k = num + Ny*Nz - Nz = num + Nz*(Ny-1)
						if((anta = cell[num+Nz*(Ny-1)]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					// plan XZ
					if(i+1 < Nx && k+1 < Nz){
						//(i+1)*Ny*Nz+j*Nz+k+1 = num + Ny*Nz + 1
						if((anta = cell[num+Ny*Nz+1]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					if(i+1 < Nx && k-1 >= 0){
						//(i+1)*Ny*Nz+j*Nz+k-1 = num + Ny*Nz - 1
						if((anta = cell[num+Ny*Nz-1]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					// plan YZ
					if(j+1 < Ny && k+1 < Nz){
						//i*Ny*Nz+(j+1)*Nz+k+1 = num + Nz + 1
						if((anta = cell[num+Nz+1]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					if(j+1 < Ny && k-1 >= 0){
						//i*Ny*Nz+(j+1)*Nz+k-1 = num + Nz - 1
						if((anta = cell[num+Nz-1]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					// tripple
					if(i+1 < Nx && j+1 < Ny && k+1 < Nz){
						//(i+1)*Ny*Nz+(j+1)*Nz+k+1 = num + Nz*(Ny + 1) + 1
						if((anta = cell[num+Nz*(Ny+1)+1]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					if(i+1 < Nx && j+1 < Ny && k-1 >= 0){
						//(i+1)*Ny*Nz+(j+1)*Nz+k-1 = num + Nz*(Ny + 1) - 1
						if((anta = cell[num+Nz*(Ny+1)-1]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					if(i+1 < Nx && j-1 >= 0 && k+1 < Nz){
						//(i+1)*Ny*Nz+(j-1)*Nz+k+1 = num + Nz*(Ny - 1) + 1
						if((anta = cell[num+Nz*(Ny-1)+1]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
					if(i-1 >= 0 && j+1 < Ny && k+1 < Nz){
						//(i-1)*Ny*Nz+(j+1)*Nz+k+1 = num + Nz*(1-Ny)+1
						if((anta = cell[num+Nz*(1-Ny)+1]) != NULL){
							do{
								for(l = 0 ; l < Ncand ; l++){
									ContactSphSph(cand[l],anta,ctl,Nctl);
								}
							}while((anta = anta->tdl) != NULL);
						}
					}
				}
			}
		}
	}
}

void ContactDetection::sphPlanContact(const int &Nsph, const int &Npl, int & Nct, vector<Sphere> & sph, vector<Plan> & pl, Contact *ct, Sphere *cell[], const double rmax) noexcept {
	Sphere *anta;
	int control[Npl];
	
	for(int i = 0 ; i < Npl ; i++){
		if(pl[i].periodic == -9)
			control[i] = 1;
		else
			control[i] = 0;
	}
	
	for(int i = 0 ; i < Npl ; i++){
		if(pl[i].periodic == -9){
			for(int j = 0 ; j < pl[i].NCell ; j++){
				if((anta = cell[pl[i].Cell[j]]) != NULL){
					do{
						ContactSphPlan(pl[i], anta, ct, Nct);
					}while((anta = anta->tdl) != NULL);
				}
			}
		}
		else{
			if(control[i] == 0){
				Sphere *llistI[Nsph];
				pl[i].Nlist = 0;
				for(int j = 0 ; j < pl[i].NCell ; j++){
					if((anta = cell[pl[i].Cell[j]]) != NULL){
						do{
							ContactSphPlanPeriodic(llistI, pl[i], pl[pl[i].periodic], anta, rmax);
						}while((anta = anta->tdl) != NULL);
					}
				}
				Sphere *llistK[Nsph];
				int k = pl[i].periodic;
				pl[k].Nlist = 0;
				for(int j = 0 ; j < pl[k].NCell ; j++){
					if((anta = cell[pl[k].Cell[j]]) != NULL){
						do{
							ContactSphPlanPeriodic(llistK, pl[k], pl[pl[k].periodic], anta, rmax);
						}while((anta = anta->tdl) != NULL);
					}
				}
				
				for(int j = 0 ; j < pl[i].Nlist ; j++){
					for(int l = 0 ; l < pl[k].Nlist ; l++){
						ContactSphSphPeriodic(llistI[j], llistK[l], ct, Nct);
					}
				}
				control[i] = 1;
				control[k] = 1;
			}
		}
	}
}

void ContactDetection::sphPlanContactOMP(const int &Nsph, const int &Npl, int & Nct, int & Ncta, vector<Sphere> & sph, vector<Plan> & pl, Contact *ct, Contact *cta, Sphere *cell[], const double rmax) noexcept {
	Sphere *anta;
	int control[Npl];
	
	for(int i = 0 ; i < Npl ; i++){
		if(pl[i].periodic == -9)
			control[i] = 1;
		else
			control[i] = 0;
	}
	for(int i = 0 ; i < Npl ; i++){
		if(pl[i].periodic == -9){
#pragma omp parallel private(anta)
			{
#pragma omp sections
				{
#pragma omp section
					{
						for(int j = 0 ; j < pl[i].NCell/2 ; j++){
							if((anta = cell[pl[i].Cell[j]]) != NULL){
								do{
									ContactSphPlan(pl[i], anta, ct, Nct);
								}while((anta = anta->tdl) != NULL);
							}
						}
					}
#pragma omp section
					{
						for(int j = pl[i].NCell/2 ; j < pl[i].NCell ; j++){
							if((anta = cell[pl[i].Cell[j]]) != NULL){
								do{
									ContactSphPlan(pl[i], anta, cta, Ncta);
								}while((anta = anta->tdl) != NULL);
							}
						}
					}
				}
			}
		}
		else{
			if(control[i] == 0){
				int k = pl[i].periodic;
				Sphere *llistK[Nsph];
				Sphere *llistI[Nsph];
#pragma omp parallel private(anta)
				{
#pragma omp sections
					{
#pragma omp section
						{
							pl[i].Nlist = 0;
							for(int j = 0 ; j < pl[i].NCell ; j++){
								if((anta = cell[pl[i].Cell[j]]) != NULL){
									do{
										ContactSphPlanPeriodic(llistI, pl[i],pl[pl[i].periodic], anta,rmax);
									}while((anta = anta->tdl) != NULL);
								}
							}
						}
#pragma omp section
						{
							pl[k].Nlist = 0;
							for(int j = 0 ; j < pl[k].NCell ; j++){
								if((anta = cell[pl[k].Cell[j]]) != NULL){
									do{
										ContactSphPlanPeriodic(llistK, pl[k],pl[pl[k].periodic], anta,rmax);
									}while((anta = anta->tdl) != NULL);
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
							for(int j = 0 ; j < pl[i].Nlist/2 ; j++){
								for(int l = 0 ; l < pl[k].Nlist ; l++){
									ContactSphSphPeriodic(llistI[j],llistK[l], ct, Nct);
								}
							}
						}
						
#pragma omp section
						{
							for(int j = pl[i].Nlist/2 ; j < pl[i].Nlist ; j++){
								for(int l = 0 ; l < pl[k].Nlist ; l++){
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

void ContactDetection::sphPlanRContact(const int &Nplr, int & Nct, vector<PlanR> & plr, Contact *ct, Sphere *cell[]) noexcept {
	Sphere *anta;
	for(int i = 0 ; i < Nplr ; i++){
		for(int j = 0 ; j < plr[i].NCell ; j++){
			if((anta = cell[plr[i].Cell[j]]) != NULL){
				do{
					ContactSphPlanR(plr[i], anta, ct, Nct);
				}while((anta = anta->tdl) != NULL);
			}
		}
	}
}

void ContactDetection::sphConeContact(const int &Nco, int & Nct, vector<Cone> & co, Contact *ct, Sphere *cell[])  noexcept {
	Sphere *anta;
	for(int i = 0 ; i < Nco ; i++){
		for(int j = 0 ; j < co[i].NCell ; j++){
			if((anta = cell[co[i].Cell[j]]) != NULL){
				do{
					ContactSphCone(co[i], anta, ct, Nct);
				}while((anta = anta->tdl) != NULL);
			}
		}
	}
}

void ContactDetection::sphElbowContact(const int &Nelb, int & Nct, vector<Elbow> & elb, Contact *ct, Sphere *cell[])  noexcept {
	Sphere *anta;
	for(int i = 0 ; i < Nelb ; i++){
		for(int j = 0 ; j < elb[i].NCell ; j++){
			if((anta = cell[elb[i].Cell[j]]) != NULL){
				do{
					ContactSphElbow(elb[i], anta, ct, Nct);
				}while((anta = anta->tdl) != NULL);
			}
		}
	}
}


void ContactDetection::sphHollowBallContact(const int &Nhb, int & Nct, vector<HollowBall> & hb, Contact *ct)  noexcept {
	for(int i = 0 ; i < Nhb ; i++){
		hb[i].ContactDetectionInHollowBall(ct,Nct);
		hb[i].ContactDetectionWithHollowBall(ct,Nct);
	}
}

void ContactDetection::sphContainer(const int & Nsph, const int & Npl, const int & Nplr, const int & Nco, const int & Nelb, const int & Nhb, vector<Sphere> & sph, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb, vector<HollowBall> & hb, int & Nct, Contact *ct, Sphere *cell[], const double rmax) noexcept {
	sphPlanContact(Nsph,Npl, Nct, sph, pl, ct, cell,rmax);
	sphPlanRContact(Nplr, Nct, plr, ct, cell);
	sphConeContact(Nco, Nct, co, ct, cell);
	sphElbowContact(Nelb, Nct, elb, ct, cell);
	sphHollowBallContact(Nhb, Nct, hb, ct);
}

void ContactDetection::sphContainerOMP(const int & Nsph, const int & Npl, const int & Nplr, const int & Nco, const int & Nelb, const int & Nhb, vector<Sphere> & sph, vector<Plan> & pl, vector<PlanR> & plr, vector<Cone> & co, vector<Elbow> & elb, vector<HollowBall> & hb, int & Nct, Contact *ct, int & Ncta, Contact *cta, Sphere *cell[], const double rmax)  noexcept {
	sphPlanContactOMP(Nsph,Npl, Nct, Ncta, sph, pl, ct, cta, cell,rmax);
	sphPlanRContact(Nplr, Nct, plr, ct, cell);
	sphConeContact(Nco, Nct, co, ct, cell);
	sphElbowContact(Nelb, Nct, elb, ct, cell);
	sphHollowBallContact(Nhb, Nct, hb, ct);
}
