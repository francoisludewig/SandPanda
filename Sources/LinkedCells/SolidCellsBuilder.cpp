#include <vector>
#include "../../Includes/LinkedCells/SolidCells.h"
#include "../../Includes/Data.h"
#include "../../Includes/LinkedCells/SolidCellsBuilder.h"
#include "../../Includes/Solids/Plan.h"
#include "../../Includes/Solids/PlanR.h"
#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Solids/Elbow.h"
#include "../../Includes/LinkedCells/CellBounds.h"

SolidCells SolidCellsBuilder::Build(const Data& dat, std::vector<Plan>& pl, std::vector<PlanR>& plr,
		std::vector<Cone>& co, std::vector<Elbow>& elb, Gravity& gt, const CellBounds& cellBounds) {
	SolidCells solidCells;
	ListCellForPlan(solidCells, dat, pl, gt, cellBounds);
	ListCellForPlanR(solidCells, dat, plr, gt, cellBounds);
	ListCellForCone(solidCells, dat, co, gt, cellBounds);
	ListCellForElbow(solidCells, dat, elb, cellBounds);
	return solidCells;
}

void SolidCellsBuilder::ListCellForPlan(SolidCells& solidCells, const Data& dat, std::vector<Plan> & pl, Gravity& gt, const CellBounds& cellBounds) noexcept {
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

	double Vmax,time,dtime,dist;
	int Nts;

	for(a = 0 ; a < pl.size() ; a++){
		std::vector<int> cell;
		if(pl[a].GetForce() == 0){
			Vmax = pl[a].Vmax();
			if(Vmax < pl[a].Wmax()*pl[a].Dt())Vmax = pl[a].Wmax()*pl[a].Dt();
			if(Vmax < pl[a].Wmax()*pl[a].Ds())Vmax = pl[a].Wmax()*pl[a].Ds();

			time = pl[a].Delay();

			dist = cellBounds.Lx()/2;
			if(dist > cellBounds.Ly()/2)dist = cellBounds.Ly()/2;
			if(dist > cellBounds.Lz()/2)dist = cellBounds.Lz()/2;

			if(Vmax != 0)
				dtime = dist/Vmax;
			else
				dtime = 0.0;

			if(time == 10000.0)time = dat.Total;

			if(time != 0)
				Nts = (int)(time/dtime)+1;
			else
				Nts = 0;

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
				dn = sqrt(cellBounds.Lx()*cellBounds.Lx()+cellBounds.Ly()*cellBounds.Ly()+cellBounds.Lz()*cellBounds.Lz());
				dt = dn;
				ds = dn;
				for(i = cellBounds.StartX(); i < cellBounds.EndX() ; i++){
					for(j = cellBounds.StartY() ; j < cellBounds.EndY() ; j++){
						for(k = cellBounds.StartZ() ; k < cellBounds.EndZ() ; k++){
							x = cellBounds.XMin() + cellBounds.Lx()*(i+0.5);
							y = cellBounds.YMin() + cellBounds.Ly()*(j+0.5);
							z = cellBounds.ZMin() + cellBounds.Lz()*(k+0.5);
							pn = (x-ox)*nx+(y-oy)*ny+(z-oz)*nz;
							pt = (x-ox)*tx+(y-oy)*ty+(z-oz)*tz;
							ps = (x-ox)*sx+(y-oy)*sy+(z-oz)*sz;
							if( (fabs(pt) < (pl[a].Dt()/2+dt)) &&
									(fabs(ps) < (pl[a].Ds()/2+ds)) &&
									(pn <= dn) && pn > -dn){
								// Control de doublon
								doublon = 0;
								num = cellBounds.Index(i, j, k);
								for(I = 0 ; I < cell.size() ; I++){
									if(num == cell[I]){
										doublon++;
									}
								}
								if(doublon == 0)
									cell.push_back(num);
							}
						}
					}
				}
			}
		} else {
			for(i = 0 ; i < cellBounds.CellCount() ; i++)
				cell.push_back(i);
		}

		printf("Ncell(%d pl) = %d\n",a,static_cast<int>(cell.size()));
		solidCells.AddPlanCells(Cells(std::move(cell)));
	}
}

void SolidCellsBuilder::ListCellForPlanR(SolidCells& solidCells, const Data& dat, std::vector<PlanR> & plr, Gravity& gt, const CellBounds& cellBounds) noexcept {
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

	double Vmax,time,dtime,dist;
	int Nts;


	for(a = 0 ;  a < plr.size() ; a++){
		std::vector<int> cell;
		if(plr[a].GetForce() == 0){

			Vmax = plr[a].Vmax();

			if(Vmax < plr[a].Wmax()*plr[a].Radius())Vmax = plr[a].Wmax()*plr[a].Radius();
			time = plr[a].Delay();

			dist = cellBounds.Lx()/2;
			if(dist > cellBounds.Ly()/2)dist = cellBounds.Ly()/2;
			if(dist > cellBounds.Lz()/2)dist = cellBounds.Lz()/2;
			if(Vmax != 0)
				dtime = dist/Vmax;
			else
				dtime = 0.0;

			if(time == 10000.0)time = dat.Total;

			if(time != 0)
				Nts = (int)(time/dtime)+1;
			else
				Nts = 0;

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
				dn = sqrt(cellBounds.Lx()*cellBounds.Lx()+cellBounds.Ly()*cellBounds.Ly()+cellBounds.Lz()*cellBounds.Lz());

				for(i = cellBounds.StartX(); i < cellBounds.EndX() ; i++){
					for(j = cellBounds.StartY() ; j < cellBounds.EndY() ; j++){
						for(k = cellBounds.StartZ() ; k < cellBounds.EndZ() ; k++){
							x = cellBounds.XMin() + cellBounds.Lx()*(i+0.5);
							y = cellBounds.YMin() + cellBounds.Ly()*(j+0.5);
							z = cellBounds.ZMin() + cellBounds.Lz()*(k+0.5);
							pn = (x-ox)*nx+(y-oy)*ny+(z-oz)*nz;
							pt = (x-ox)*tx+(y-oy)*ty+(z-oz)*tz;
							ps = (x-ox)*sx+(y-oy)*sy+(z-oz)*sz;
							if( (sqrt(pt*pt+ps*ps) < plr[a].Radius()+2*dn) &&
									(pn >= -dn) && (pn < 2*dn)){
								// Control de doublon
								doublon = 0;
								num = cellBounds.Index(i, j, k);
								for(I = 0 ; I < cell.size() ; I++){
									if(num == cell[I]){
										doublon++;
									}
								}
								if(doublon == 0)
									cell.push_back(num);
							}
						}
					}
				}
			}

		} else {
			for(i = 0 ; i < cellBounds.CellCount() ; i++)
				cell.push_back(i);
		}
		printf("Ncell(%d plr) = %d\n",a,static_cast<int>(cell.size()));
		solidCells.AddPlanRCells(Cells(std::move(cell)));
	}
}

void SolidCellsBuilder::ListCellForCone(SolidCells& solidCells, const Data& dat, std::vector<Cone> & co, Gravity& gt, const CellBounds& cellBounds) noexcept {
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
	double N,dr,h,b,Y,delta2,beta,a;

	double Vmax,time,dtime,dist;
	int Nts,b2;

	for(numCone = 0 ;  numCone < co.size() ; numCone++) {
		std::vector<int> cell;

		if(co[numCone].GetForce() == 0) {
			Vmax = co[numCone].Vmax();
			if(Vmax < co[numCone].Wmax()*co[numCone].Height()/2.)Vmax = co[numCone].Wmax()*co[numCone].Height()/2.;
			time = co[numCone].Delay();

			dist = cellBounds.Lx()/2;
			if(dist > cellBounds.Ly()/2)dist = cellBounds.Ly()/2;
			if(dist > cellBounds.Lz()/2)dist = cellBounds.Lz()/2;

			if(Vmax != 0)
				dtime = dist/Vmax;
			else
				dtime = 0.0;
			if(time == 10000.0)time = dat.Total;

			if(time != 0)
				Nts = (int)(time/dtime)+1;
			else
				Nts = 0;


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
				dn = sqrt(cellBounds.Lx()*cellBounds.Lx()+cellBounds.Ly()*cellBounds.Ly()+cellBounds.Lz()*cellBounds.Lz());

				for(i = cellBounds.StartX(); i < cellBounds.EndX() ; i++){
					for(j = cellBounds.StartY() ; j < cellBounds.EndY() ; j++){
						for(k = cellBounds.StartZ() ; k < cellBounds.EndZ() ; k++){
							px = cellBounds.XMin() + cellBounds.Lx()*(i+0.5);
							py = cellBounds.YMin() + cellBounds.Ly()*(j+0.5);
							pz = cellBounds.ZMin() + cellBounds.Lz()*(k+0.5);
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
										num = cellBounds.Index(i, j, k);
										for(I = 0 ; I < cell.size() ; I++){
											if(num == cell[I]){
												doublon++;
											}
										}
										if(doublon == 0)
											cell.push_back(num);
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
										num = cellBounds.Index(i, j, k);
										for(I = 0 ; I < cell.size() ; I++){
											if(num == cell[I]){
												doublon++;
											}
										}
										if(doublon == 0){
											cell.push_back(num);
										}
									}
								}
							}
						}

					}
				}
			}
		} else {
			for(int i = 0 ; i < cellBounds.CellCount() ; i++)
				cell.push_back(i);
		}
		printf("Ncell(%d co) = %d\n",numCone,static_cast<int>(cell.size()));
		solidCells.AddConeCells(Cells(std::move(cell)));
	}
}

void SolidCellsBuilder::ListCellForElbow(SolidCells& solidCells, const Data& dat, std::vector<Elbow> & el, const CellBounds& cellBounds) noexcept {
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

	double Vmax,time,dtime,dist;
	int Nts,b;
	double rl, alphal,cx,cy,cz,D;

	for(numEl = 0 ;  numEl < el.size() ; numEl++){
		std::vector<int> cell;
		Vmax = el[numEl].Vmax();
		if(Vmax < el[numEl].Wmax()*(el[numEl].r+el[numEl].R))Vmax = el[numEl].Wmax()*(el[numEl].r+el[numEl].R);
		time = el[numEl].Delay();

		dist = cellBounds.Lx()/2;
		if(dist > cellBounds.Ly()/2)dist = cellBounds.Ly()/2;
		if(dist > cellBounds.Lz()/2)dist = cellBounds.Lz()/2;
		if(Vmax != 0)
			dtime = dist/Vmax;
		else
			dtime = 0.0;
		if(time == 10000.0)time = dat.Total;

		if(time != 0)
			Nts = (int)(time/dtime)+1;
		else
			Nts = 0;

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
			dn = sqrt(cellBounds.Lx()*cellBounds.Lx()+cellBounds.Ly()*cellBounds.Ly()+cellBounds.Lz()*cellBounds.Lz());
			for(i = cellBounds.StartX(); i < cellBounds.EndX() ; i++){
				for(j = cellBounds.StartY() ; j < cellBounds.EndY() ; j++){
					for(k = cellBounds.StartZ() ; k < cellBounds.EndZ() ; k++){
						px = cellBounds.XMin() + cellBounds.Lx()*(i+0.5);
						py = cellBounds.YMin() + cellBounds.Ly()*(j+0.5);
						pz = cellBounds.ZMin() + cellBounds.Lz()*(k+0.5);
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
									num = cellBounds.Index(i, j, k);
									for(I = 0 ; I < cell.size() ; I++){
										if(num == cell[I]){
											doublon++;
										}
									}
									if(doublon == 0)
										cell.push_back(num);
								}
							}
						}
					}
				}
			}
		}
		printf("Ncell(%d el) = %d\n",numEl,static_cast<int>(cell.size()));
		solidCells.AddElbowCells(Cells(std::move(cell)));

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
}
