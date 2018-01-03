#include "../Includes/Periodicity.h"

#import "../Includes/Plan.h"
#import "../Includes/Sphere.h"
#import "../Includes/Body.h"

void PeriodicityPL(const int & Nsph, const int & Nbd, const int &Npl, vector<Sphere> & sph, vector<Body> & bd, vector<Plan> & pl) noexcept {	
	int plp;
	double px,py,pz,pn,pt,ps;
	Sphere *b;
	
	for(int i = 0 ; i < Npl ; i++){
		if((plp = pl[i].periodic) != -9){
			for(int j = 0 ; j < pl[i].Nlist ; j++){
				b = &sph[pl[i].list[j]];
				if((b->Bodies()) == -9){
					px = b->X();
					py = b->Y();
					pz = b->Z();
					pn = (px-pl[i].X())*pl[i].nx+(py-pl[i].Y())*pl[i].ny+(pz-pl[i].Z())*pl[i].nz;
					pt = (px-pl[plp].X())*pl[plp].tx+(py-pl[plp].Y())*pl[plp].ty+(pz-pl[plp].Z())*pl[plp].tz;
					ps = (px-pl[plp].X())*pl[plp].sx+(py-pl[plp].Y())*pl[plp].sy+(pz-pl[plp].Z())*pl[plp].sz;
					if(pn < 0){
						b->X(pl[plp].X() + (-pn)*pl[plp].nx + pt*pl[plp].tx + ps*pl[plp].sx);
						b->Y(pl[plp].Y() + (-pn)*pl[plp].ny + pt*pl[plp].ty + ps*pl[plp].sy);
						b->Z(pl[plp].Z() + (-pn)*pl[plp].nz + pt*pl[plp].tz + ps*pl[plp].sz);
					}
				}
			} 
		}
	}
	
	for(int i = 0 ; i < Npl ; i++){
		if((plp = pl[i].periodic) != -9){
			for(int j = 0 ; j < pl[i].Nlist ; j++){
				b = &sph[pl[i].list[j]];	
				if(b->Bodies() != -9){
					px = (b->GetBody())->X();
					py = (b->GetBody())->Y();
					pz = (b->GetBody())->Z();
					pn = (px-pl[i].X())*pl[i].nx+(py-pl[i].Y())*pl[i].ny+(pz-pl[i].Z())*pl[i].nz;
					pt = (px-pl[plp].X())*pl[plp].tx+(py-pl[plp].Y())*pl[plp].ty+(pz-pl[plp].Z())*pl[plp].tz;
					ps = (px-pl[plp].X())*pl[plp].sx+(py-pl[plp].Y())*pl[plp].sy+(pz-pl[plp].Z())*pl[plp].sz;
					if(pn < 0){
						(b->GetBody())->X(pl[plp].X() + (-pn)*pl[plp].nx + pt*pl[plp].tx + ps*pl[plp].sx);
						(b->GetBody())->Y(pl[plp].Y() + (-pn)*pl[plp].ny + pt*pl[plp].ty + ps*pl[plp].sy);
						(b->GetBody())->Z(pl[plp].Z() + (-pn)*pl[plp].nz + pt*pl[plp].tz + ps*pl[plp].sz);
						(b->GetBody())->UpDateLinkedSphere(sph);
					}
				}
			}
		}
	}
}
