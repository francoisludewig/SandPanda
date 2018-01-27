#include "../../Includes/Contact/ContactDetectorSphereVersusCone.h"

#include "../../Includes/Contact/Contact.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Contact/ContactDetectorBodyVersusCone.h"

void ContactDetectorSphereVersusCone::Detect(Cone & p, Sphere *b, Contact *ct, int & Nct) noexcept {
	if(b->Bodies() != -9){
		ContactDetectorBodyVersusCone::Detect(p,b->GetBody(),ct,Nct);
		return;
	}

	double px = b->X();
	double py = b->Y();
	double pz = b->Z();
	double pn = (px-p.X())*p.Nx()+(py-p.Y())*p.Ny()+(pz-p.Z())*p.Nz();
	double pt = (px-p.X())*p.Tx()+(py-p.Y())*p.Ty()+(pz-p.Z())*p.Tz();
	double ps = (px-p.X())*p.Sx()+(py-p.Y())*p.Sy()+(pz-p.Z())*p.Sz();

	if(fabs(pn) <= p.Height()/2 + b->Radius()){
		double N = sqrt(pt*pt+ps*ps);
		double Tx = (pt*p.Tx() + ps*p.Sx())/N;
		double Ty = (pt*p.Ty() + ps*p.Sy())/N;
		double Tz = (pt*p.Tz() + ps*p.Sz())/N;
		double Nx = p.Nx();
		double Ny = p.Ny();
		double Nz = p.Nz();
		double dr = p.BottomRadius() -  p.TopRadius();
		double h = p.Height();

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
				double nnx = px-b->X();
				double nny = py-b->Y();
				double nnz = pz-b->Z();
				double nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
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

			double a = -h/dr;
			double bb = h*(p.BottomRadius()/dr-0.5);
			double Y = sqrt(dr*dr+h*h);
			double delta2 = -(N - pn/a + bb/a)*h/Y;

			if(fabs(delta2) <= b->Radius()){
				double Qn = pn + delta2*dr/Y;
				double Qt = N + delta2*h/Y;
				px = p.X() + Qn*Nx + Qt*Tx;
				py = p.Y() + Qn*Ny + Qt*Ty;
				pz = p.Z() + Qn*Nz + Qt*Tz;
				if(Qn < h/2. && Qn > -h/2.){
					ct[Nct].delta = fabs(delta2) - b->Radius();
					ct[Nct].type = Contact::Type::SphereCone;// 3;
					ct[Nct].px = px;
					ct[Nct].py = py;
					ct[Nct].pz = pz;
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
					double nnx = px-b->X();
					double nny = py-b->Y();
					double nnz = pz-b->Z();
					double nn = sqrt(nnx*nnx+nny*nny+nnz*nnz);
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
