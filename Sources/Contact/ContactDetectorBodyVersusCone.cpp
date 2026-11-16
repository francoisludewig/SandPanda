#include "../../Includes/Contact/ContactDetectorBodyVersusCone.h"

#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Solids/Cone.h"

void ContactDetectorBodyVersusCone::Detect(Cone & p, Body *b, Contact *ct, int & Nct) noexcept {
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
