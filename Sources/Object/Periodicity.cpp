#include "../../Includes/Object/Periodicity.h"
#include "../../Includes/Object/Plan.h"
#include "../../Includes/Object/Sphere.h"
#include "../../Includes/Object/Body.h"

void PeriodicityPL(const int & Nsph, const int & Nbd, const int &Npl, vector<Sphere> & sph, vector<Body> & bd, vector<Plan> & pl) noexcept {	
	int plp;
	double px,py,pz,pn,pt,ps;
	Sphere *b;
	
	for(int i = 0 ; i < Npl ; i++){
		if((plp = pl[i].periodic) != -9){
			for(int j = 0 ; j < pl[i].Nlist ; j++){
				b = &sph[pl[i].list[j]];
				if((b->bodies) == -9){
					px = b->x;
					py = b->y;
					pz = b->z;
					pn = (px-pl[i].x)*pl[i].nx+(py-pl[i].y)*pl[i].ny+(pz-pl[i].z)*pl[i].nz;
					pt = (px-pl[plp].x)*pl[plp].tx+(py-pl[plp].y)*pl[plp].ty+(pz-pl[plp].z)*pl[plp].tz;
					ps = (px-pl[plp].x)*pl[plp].sx+(py-pl[plp].y)*pl[plp].sy+(pz-pl[plp].z)*pl[plp].sz;
					if(pn < 0){
						b->x = pl[plp].x + (-pn)*pl[plp].nx + pt*pl[plp].tx + ps*pl[plp].sx;
						b->y = pl[plp].y + (-pn)*pl[plp].ny + pt*pl[plp].ty + ps*pl[plp].sy;
						b->z = pl[plp].z + (-pn)*pl[plp].nz + pt*pl[plp].tz + ps*pl[plp].sz;
					}
				}
			} 
		}
	}
	
	for(int i = 0 ; i < Npl ; i++){
		if((plp = pl[i].periodic) != -9){
			for(int j = 0 ; j < pl[i].Nlist ; j++){
				b = &sph[pl[i].list[j]];	
				if(b->bodies != -9){
					px = (b->b)->x;
					py = (b->b)->y;
					pz = (b->b)->z;
					pn = (px-pl[i].x)*pl[i].nx+(py-pl[i].y)*pl[i].ny+(pz-pl[i].z)*pl[i].nz;
					pt = (px-pl[plp].x)*pl[plp].tx+(py-pl[plp].y)*pl[plp].ty+(pz-pl[plp].z)*pl[plp].tz;
					ps = (px-pl[plp].x)*pl[plp].sx+(py-pl[plp].y)*pl[plp].sy+(pz-pl[plp].z)*pl[plp].sz;
					if(pn < 0){
						(b->b)->x = pl[plp].x + (-pn)*pl[plp].nx + pt*pl[plp].tx + ps*pl[plp].sx;
						(b->b)->y = pl[plp].y + (-pn)*pl[plp].ny + pt*pl[plp].ty + ps*pl[plp].sy;
						(b->b)->z = pl[plp].z + (-pn)*pl[plp].nz + pt*pl[plp].tz + ps*pl[plp].sz;
						(b->b)->UpDateLinkedSphere(sph);
					}
				}
			}
		}
	}
}
