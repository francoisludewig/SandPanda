#include "../../Includes/LinkedCells/LinkedCellSolidListBuilder.h"

#include <vector>

#include "../../Includes/Data.h"
#include "../../Includes/Solids/Plan.h"
#include "../../Includes/Solids/PlanR.h"
#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Solids/Elbow.h"

void LinkedCellSolidListBuilder::ListCellForPlan(const Data& dat, std::vector<Plan> & pl, Gravity& gt) noexcept {
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
	localCell =(int*)malloc(dat.Nx*dat.Ny*dat.Nz*sizeof(int));

	for(a = 0 ; a < pl.size() ; a++){
		if(pl[a].GetForce() == 0){
			Vmax = pl[a].Vmax();
			if(Vmax < pl[a].Wmax()*pl[a].Dt())Vmax = pl[a].Wmax()*pl[a].Dt();
			if(Vmax < pl[a].Wmax()*pl[a].Ds())Vmax = pl[a].Wmax()*pl[a].Ds();

			time = pl[a].Delay();

			dist = dat.ax/2;
			if(dist > dat.ay/2)dist = dat.ay/2;
			if(dist > dat.az/2)dist = dat.az/2;

			if(Vmax != 0)
				dtime = dist/Vmax;
			else
				dtime = 0.0;

			if(time == 10000.0)time = dat.Total;

			if(time != 0)
				Nts = (int)(time/dtime)+1;
			else
				Nts = 0;


			localNcell = 0;
			for(i = dat.Nx*dat.Ny*dat.Nz ; i--;){
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
				dn = sqrt(dat.ax*dat.ax+dat.ay*dat.ay+dat.az*dat.az);
				dt = dn;
				ds = dn;
				for(i = 0 ; i < dat.Nx ; i++){
					for(j = 0 ; j < dat.Ny ; j++){
						for(k = 0 ; k < dat.Nz ; k++){
							x = dat.xmin + dat.ax*(i+0.5);
							y = dat.ymin + dat.ay*(j+0.5);
							z = dat.zmin + dat.az*(k+0.5);
							pn = (x-ox)*nx+(y-oy)*ny+(z-oz)*nz;
							pt = (x-ox)*tx+(y-oy)*ty+(z-oz)*tz;
							ps = (x-ox)*sx+(y-oy)*sy+(z-oz)*sz;
							if( (fabs(pt) < (pl[a].Dt()/2+dt)) &&
							   (fabs(ps) < (pl[a].Ds()/2+ds)) &&
							   (pn <= dn) && pn > -dn){
								// Control de doublon
								doublon = 0;
								num = (i*dat.Ny*dat.Nz+j*dat.Nz+k);
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
			pl[a].Cell = (int*)malloc(dat.Nx*dat.Ny*dat.Nz*sizeof(int));
			pl[a].NCell = dat.Nx*dat.Ny*dat.Nz;
			printf("Ncell(%d pl) = %d\n",a,pl[a].NCell);
			for(i = 0 ; i < dat.Nx*dat.Ny*dat.Nz ; i++){
				pl[a].Cell[i] = i;
			}
		}
	}
	free(localCell);
}

void LinkedCellSolidListBuilder::ListCellForPlanR(const Data& dat, std::vector<PlanR> & plr, Gravity& gt) noexcept {
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

	localCell =(int*)malloc(dat.Nx*dat.Ny*dat.Nz*sizeof(int));

	for(a = 0 ;  a < plr.size() ; a++){

		if(plr[a].GetForce() == 0){

			Vmax = plr[a].Vmax();

			if(Vmax < plr[a].Wmax()*plr[a].Radius())Vmax = plr[a].Wmax()*plr[a].Radius();
			time = plr[a].Delay();

			dist = dat.ax/2;
			if(dist > dat.ay/2)dist = dat.ay/2;
			if(dist > dat.az/2)dist = dat.az/2;
			if(Vmax != 0)
				dtime = dist/Vmax;
			else
				dtime = 0.0;

			if(time == 10000.0)time = dat.Total;

			if(time != 0)
				Nts = (int)(time/dtime)+1;
			else
				Nts = 0;

			localNcell = 0;
			for(i = dat.Nx*dat.Ny*dat.Nz ; i--;){
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
				dn = sqrt(dat.ax*dat.ax+dat.ay*dat.ay+dat.az*dat.az);

				for(i = 0 ; i < dat.Nx ; i++){
					for(j = 0 ; j < dat.Ny ; j++){
						for(k = 0 ; k < dat.Nz ; k++){
							x = dat.xmin + dat.ax*(i+0.5);
							y = dat.ymin + dat.ay*(j+0.5);
							z = dat.zmin + dat.az*(k+0.5);
							pn = (x-ox)*nx+(y-oy)*ny+(z-oz)*nz;
							pt = (x-ox)*tx+(y-oy)*ty+(z-oz)*tz;
							ps = (x-ox)*sx+(y-oy)*sy+(z-oz)*sz;
							if( (sqrt(pt*pt+ps*ps) < plr[a].Radius()+2*dn) &&
							   (pn >= -dn) && (pn < 2*dn)){
								// Control de doublon
								doublon = 0;
								num = (i*dat.Ny*dat.Nz+j*dat.Nz+k);
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
			plr[a].Cell = (int*)malloc(dat.Nx*dat.Ny*dat.Nz*sizeof(int));
			plr[a].NCell = dat.Nx*dat.Ny*dat.Nz;
			printf("Ncell(%d plr) = %d\n",a,plr[a].NCell);
			for(i = 0 ; i < dat.Nx*dat.Ny*dat.Nz ; i++){
				plr[a].Cell[i] = i;
			}

		}
	}
	free(localCell);
}

void LinkedCellSolidListBuilder::ListCellForCone(const Data& dat, std::vector<Cone> & co, Gravity& gt) noexcept {
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


	localCell =(int*)malloc(dat.Nx*dat.Ny*dat.Nz*sizeof(int));

	for(numCone = 0 ;  numCone < co.size() ; numCone++){
		if(co[numCone].GetForce() == 0){
			Vmax = co[numCone].Vmax();
			if(Vmax < co[numCone].Wmax()*co[numCone].Height()/2.)Vmax = co[numCone].Wmax()*co[numCone].Height()/2.;
			time = co[numCone].Delay();

			dist = dat.ax/2;
			if(dist > dat.ay/2)dist = dat.ay/2;
			if(dist > dat.az/2)dist = dat.az/2;

			if(Vmax != 0)
				dtime = dist/Vmax;
			else
				dtime = 0.0;
			if(time == 10000.0)time = dat.Total;

			if(time != 0)
				Nts = (int)(time/dtime)+1;
			else
				Nts = 0;



			localNcell = 0;
			for(i = dat.Nx*dat.Ny*dat.Nz ; i--;){
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
				if(dat.Nx > 1)
					dn = sqrt(dat.ax*dat.ax+dat.ay*dat.ay+dat.az*dat.az);
				else{
					dn = sqrt(dat.ay*dat.ay+dat.az*dat.az);

				}
				for(i = 0 ; i < dat.Nx ; i++){
					for(j = 0 ; j < dat.Ny ; j++){
						for(k = 0 ; k < dat.Nz ; k++){
							px = dat.xmin + dat.ax*(i+0.5);
							py = dat.ymin + dat.ay*(j+0.5);
							pz = dat.zmin + dat.az*(k+0.5);
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
										num = (i*dat.Ny*dat.Nz+j*dat.Nz+k);
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
										num = (i*dat.Ny*dat.Nz+j*dat.Nz+k);
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
			co[numCone].Cell = (int*)malloc(dat.Nx*dat.Ny*dat.Nz*sizeof(int));
			co[numCone].NCell = dat.Nx*dat.Ny*dat.Nz;
			printf("Ncell(%d plr) = %d\n",numCone,co[numCone].NCell);
			for(i = 0 ; i < dat.Nx*dat.Ny*dat.Nz ; i++){
				co[numCone].Cell[i] = i;
			}

		}
	}
	free(localCell);

}

void LinkedCellSolidListBuilder::ListCellForElbow(const Data& dat, std::vector<Elbow> & el) noexcept {
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

	localCell =(int*)malloc(dat.Nx*dat.Ny*dat.Nz*sizeof(int));

	for(numEl = 0 ;  numEl < el.size() ; numEl++){
		Vmax = el[numEl].Vmax();
		if(Vmax < el[numEl].Wmax()*(el[numEl].r+el[numEl].R))Vmax = el[numEl].Wmax()*(el[numEl].r+el[numEl].R);
		time = el[numEl].Delay();

		dist = dat.ax/2;
		if(dist > dat.ay/2)dist = dat.ay/2;
		if(dist > dat.az/2)dist = dat.az/2;
		if(Vmax != 0)
			dtime = dist/Vmax;
		else
			dtime = 0.0;
		if(time == 10000.0)time = dat.Total;

		if(time != 0)
			Nts = (int)(time/dtime)+1;
		else
			Nts = 0;

		localNcell = 0;
		for(i = dat.Nx*dat.Ny*dat.Nz ; i--;){
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
			dn = sqrt(dat.ax*dat.ax+dat.ay*dat.ay+dat.az*dat.az);
			for(i = 0 ; i < dat.Nx ; i++){
				for(j = 0 ; j < dat.Ny ; j++){
					for(k = 0 ; k < dat.Nz ; k++){
						px = dat.xmin + dat.ax*(i+0.5);
						py = dat.ymin + dat.ay*(j+0.5);
						pz = dat.zmin + dat.az*(k+0.5);
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
									num = (i*dat.Ny*dat.Nz+j*dat.Nz+k);
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
