#include "../Includes/HollowBall.h"
#include "../Includes/Sphere.h"
#include "../Includes/ContactDetection.h"
#include "../Includes/Contact.h"
#include "../Includes/Body.h"

#include <cmath>

HollowBall::HollowBall() noexcept {
    x = 0;
    y = 0;
    z = 0;
    vx = 0;
    vy = 0;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    q0 = 1;
    q1 = 0;
    q2 = 0;
    q3 = 0;
    mass = 1;
    Inertia = 10;
    r = 2;
    cell = NULL;
    list = NULL;
    inSph = NULL;
    lockVx = false;
    lockVy = false;
    lockVz = false;
    lockWx = false;
    lockWy = false;
    lockWz = false;
}

HollowBall::~HollowBall() noexcept {
    if(inSph != NULL)
        delete[] inSph;
    if(cell != NULL)
       delete[] cell;
    if(list != NULL)
        delete[] list;
}

void HollowBall::loadFromFile(FILE *ft) noexcept {
    int b1,b2,b3,b4,b5,b6;
    fscanf(ft,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%d\t%d\t%d\t%d\t%d\t%d\t%lf\t%lf\t%lf",&x,&y,&z,&q0,&q1,&q2,&q3,&vx,&vy,&vz,&wx,&wy,&wz,&b1,&b2,&b3,&b4,&b5,&b6,&r,&mass,&Inertia);
    lockVx = b1;
    lockVy = b2;
    lockVz = b3;
    lockWx = b4;
    lockWy = b5;
    lockWz = b6;
}

void HollowBall::writeToFile(FILE *ft) const noexcept {
    fprintf(ft,"%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%e\t%d\t%d\t%d\t%d\t%d\t%d\t%e\t%e\t%e\n",x,y,z,q0,q1,q2,q3,vx,vy,vz,wx,wy,wz,lockVx,lockVy,lockVz,lockWx,lockWy,lockWz,r,mass,Inertia);
}

void HollowBall::Makeavatar(vector<Sphere> & sph, int & Nsph, const int numero) noexcept {
    Sphere *sphl = new Sphere();
	sphl->x = x;
	sphl->y = y;
	sphl->z = z;
	sphl->r = r;
    sphl->q0 = q0;
	sphl->q1 = q1;
	sphl->q2 = q2;
	sphl->q3 = q3;
    
	sphl->bodies = -9;
	sphl->num = Nsph;
    sphl->m = mass;
    sphl->I = Inertia;
    sphl->isHollowBall = true;
	//sphl->NhollowBall = numero;
	sph.push_back(*sphl);
	delete sphl;
	Nsph++;
    Navatar = Nsph-1;
}

void HollowBall::LinkInSph(vector<Sphere> & sph, int & Nsph, const int numero) noexcept {
    avatar = &sph[Navatar];
    NinSph = 0;
    for(int i = 0 ; i < Nsph ; i++){
        if(sph[i].HollowballNum() == numero)
            NinSph++;
    }
    inSph = new Sphere*[NinSph];
    
    NinSph = 0;
    for(int i = 0 ; i < Nsph ; i++){
        if(sph[i].HollowballNum() == numero){
            inSph[NinSph] = &sph[i];
            NinSph++;
        }
    }
    
    /*
    for(int i = 0 ; i < NinSph ; i++){
        cout << i << "\t" << inSph[i] << endl;
    }
    */
    
    if(NinSph != 0){
        // Determination des cellules liees de la HollowBall
        rmax = inSph[0]->Radius();
        for(int i = 1 ; i < NinSph ; i++){
            if(rmax < inSph[i]->Radius())
                rmax = inSph[i]->Radius();
        }
        a = 2*rmax*1.01;
        N = (int)(2*r/a);
        a = 2*r/N;
        cell = new Sphere*[N*N*N];
        
        printf("Ninsph = %d\n",NinSph);
        printf("rmax = %e\n",rmax);
        printf("N = %d\n",N);
        
        // Fabrication de la liste des cellules liees pour la detection des contacts entre la HB et les spheres
        double xl,yl,zl;
        int Ncand = 0;
        for(int i = 0 ; i < N ; i++){
            xl = -r + i*a + a/2.;
            for(int j = 0 ; j < N ; j++){
                yl = -r + j*a + a/2.;
                for(int k = 0 ; k < N ; k++){
                    zl = -r + k*a + a/2.;
                    if(sqrt(xl*xl+yl*yl+zl*zl) > r-sqrt(3.)*a && sqrt(xl*xl+yl*yl+zl*zl) < r+sqrt(3.)*a)
                        Ncand++;
                }
            }
        }
        Nlist = Ncand;
        list = new int[Ncand];
        Ncand = 0;
        for(int i = 0 ; i < N ; i++){
            xl = -r + i*a + a/2.;
            for(int j = 0 ; j < N ; j++){
                yl = -r + j*a + a/2.;
                for(int k = 0 ; k < N ; k++){
                    zl = -r + k*a + a/2.;
                    if(sqrt(xl*xl+yl*yl+zl*zl) > r-sqrt(3.)*a && sqrt(xl*xl+yl*yl+zl*zl) < r+sqrt(3.)*a){
                        list[Ncand] = i*N*N+j*N+k;
                        Ncand++;
                    }
                }
            }
        }
    }
    /*
     for(int i = 0 ; i < Nlist ; i++){
     printf("L(%d) = %d\n",i,list[i]);
     }
     */
}

void HollowBall::ContactDetectionInHollowBall(Contact *ct, int & Nct) noexcept {
    int nx,ny,nz;
    // Determination des minima fonction de la position de la HollowBall
    xmin = x-r;
    ymin = y-r;
    zmin = z-r;
    //printf("Detection in HB (%d)\n",NinSph);

    
    for(int i = 0 ; i < NinSph ; i++)
        (*inSph[i]).tdl = NULL;
     
    for(int i = 0 ; i < N*N*N ; i++)
        cell[i] = NULL;
    
    // Classification des spheres dans les cellules
    for(int i = 0 ; i < NinSph ; i++){
        nx = (int)(((*inSph[i]).x-xmin)/a);
        ny = (int)(((*inSph[i]).y-ymin)/a);
        nz = (int)(((*inSph[i]).z-zmin)/a);
        if(nx < N && ny < N && nz < N && nx >= 0 && ny >= 0 && nz >= 0){
            (*inSph[i]).tdl = cell[nx*N*N+ny*N+nz];
            cell[nx*N*N+ny*N+nz] = inSph[i];
        }
    }
	ContactDetection::sphContact(0,N,N,0,N,N,N,ct,Nct,cell);
}

void HollowBall::ContactDetectionWithHollowBall(Contact *ct, int & Nct) noexcept {
    Sphere *cand;
    Body *candb;
    double dx,dy,dz,D;
    //  printf("Detection with HB\n");

    for(int i = 0 ; i < Nlist ; i++){
        cand = cell[list[i]];
        if(cand != NULL){
            do{
                if((candb = cand->b) == NULL){
                    // Test du contact entre la sphere cand et la HollowBall
                    dx = x - (cand->x);
                    dy = y - (cand->y);
                    dz = z - (cand->z);
                    if((D = sqrt(dx*dx+dy*dy+dz*dz)) > (r-cand->r)){
                        ct[Nct].type = 5;
                        ct[Nct].delta = (r-cand->r)-D;
                        ct[Nct].nx = dx/D;
                        ct[Nct].ny = dy/D;
                        ct[Nct].nz = dz/D;
                        ct[Nct].px = cand->x - dx/D*cand->r;
                        ct[Nct].py = cand->y - dy/D*cand->r;
                        ct[Nct].pz = cand->z - dz/D*cand->r;
                        ct[Nct].sa = cand;
                        ct[Nct].sb = avatar;
                        Nct++;
                    }
                }
                // Test du contact entre la particule cand et la HollowBall
                else{
                   
                    for(int j = candb->Ng-1 ; j--;){
                        // Test du contact entre la sphere cand et la HollowBall
                        dx = x - (candb->xg[j]);
                        dy = y - (candb->yg[j]);
                        dz = z - (candb->zg[j]);
                        if((D = sqrt(dx*dx+dy*dy+dz*dz)) > (r-candb->r[j])){
                           // printf("Contact HollowBall and Particle %d (Ng = %d)(d = %e)\n\n",candb->Num(),j,(r-candb->r[j])-D);
                           // printf("sph(%d) = (%e,%e,%e)\n",j,candb->xg[j],candb->yg[j],candb->zg[j]);
                           // exit(0);
                            ct[Nct].type = 6;
                            ct[Nct].delta = (r-candb->r[j])-D;
                            ct[Nct].nx = dx/D;
                            ct[Nct].ny = dy/D;
                            ct[Nct].nz = dz/D;
                            ct[Nct].px = candb->xg[j] - dx/D*candb->r[j];
                            ct[Nct].py = candb->yg[j] - dy/D*candb->r[j];
                            ct[Nct].pz = candb->zg[j] - dz/D*candb->r[j];
                            ct[Nct].sa = avatar;
                            ct[Nct].bb = candb;
                            ct[Nct].nbb = j;
                            Nct++;
                        }
                    }
                }
            }while((cand = cand->tdl) != NULL);
        }
    }
}

void HollowBall::UpdateFromSph(double dt) noexcept {
    double a,ca,sa,p0,p1,p2,p3,ql0,ql1,ql2,ql3;
    
    if(!lockVx){
        x = avatar->x;
        vx = avatar->vx;
    }
    else{
        avatar->x = x;
        avatar->vx = vx;
    }
    if(!lockVy){
        y = avatar->y;
        vy = avatar->vy;
    }
    else{
        avatar->y = y;
        avatar->vy = vy;
    }
    if(!lockVz){
        z = avatar->z;
        vz = avatar->vz;
    }
    else{
        avatar->z = z;
        avatar->vz = vz;
    }
    if(!lockWx)
        wx = avatar->wx;
    else
        avatar->wx = wx;
    if(!lockWy)
        wy = avatar->wy;
    else
        avatar->wy = wy;
    if(!lockWz)
        wz = avatar->wz;
    else
        avatar->wz = wz;
    
    a = sqrt(wx*wx+wy*wy+wz*wz)*dt;
    if(a != 0){
        sa = sin(a/2);
        ca = cos(a/2);
        p0 = ca;
        p1 = dt*wx/a*sa;
        p2 = dt*wy/a*sa;
        p3 = dt*wz/a*sa;
        
        ql0 = q0;
        ql1 = q1;
        ql2 = q2;
        ql3 = q3;
        
        //if(p0 != 0 && ql0 != 0){
            q0 = ql0*p0 - ql1*p1 - ql2*p2 - ql3*p3;
            q1 = ql0*p1 + ql1*p0 - ql2*p3 + ql3*p2;
            q2 = ql0*p2 + ql1*p3 + ql2*p0 - ql3*p1;
            q3 = ql0*p3 - ql1*p2 + ql2*p1 + ql3*p0;
        /*
        }
        if(p0 != 0 && ql0 == 0){
            q0 = p0;
            q1 = p1;
            q2 = p2;
            q3 = p3;
        }
         */
    }
}

