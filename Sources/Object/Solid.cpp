#include "../../Includes/Object/Solid.h"
#include "../../Includes/Object/Sphere.h"

#include <cmath>

Solid::Solid() noexcept {
    nx = 1;
    ny = 0;
    nz = 0;
    tx = 0;
    ty = 1;
    tz = 0;
    sx = 0;
    sy = 0;
    sz = 1;
    Fcx = 0;
    Fcy = 0;
    Fcz = 0;
    Mcx = 0;
    Mcy = 0;
    Mcz = 0;
    xMemory = 0;
    yMemory = 0;
    zMemory = 0;
    Mass = 1;
    In = 1;
    It = 1;
    Is = 1;
    activeGravity = 0;
    //NCell = 0;
    Force = 0;
    NCell = 0;
    Cell = nullptr;
}

Solid::~Solid() noexcept {

}

void Solid::LoadFromFile(FILE *ft) noexcept {
    fscanf(ft,"%lf\t%lf\t%lf",&x,&y,&z);
    fscanf(ft,"%lf\t%lf\t%lf\t%lf",&q0,&q1,&q2,&q3);
    ComputeBase();
    V.LoadFromFile(ft);
}

void Solid::ComputeBase() noexcept {
    nx = 1 - 2*q2*q2 - 2*q3*q3;
    ny = 2*q1*q2 + 2*q3*q0;
    nz = 2*q1*q3 - 2*q2*q0;

    tx = 2*q1*q2 - 2*q3*q0;
    ty = 1 - 2*q1*q1 - 2*q3*q3;
    tz = 2*q2*q3 + 2*q1*q0;

    sx = 2*q1*q3 + 2*q2*q0;
    sy = 2*q2*q3 - 2*q1*q0;
    sz = 1 - 2*q1*q1 - 2*q2*q2;

    // Elimination des erreurs
    double max = fabs(nx);
    if(max < fabs(ny))max = fabs(ny);
    if(max < fabs(nz))max = fabs(nz);

    if(fabs(nx)/max < pow(10.,-15))nx = 0;
    if(fabs(ny)/max < pow(10.,-15))ny = 0;
    if(fabs(nz)/max < pow(10.,-15))nz = 0;

    max = fabs(tx);
    if(max < fabs(ty))max = fabs(ty);
    if(max < fabs(tz))max = fabs(tz);

    if(fabs(tx)/max < pow(10.,-15))tx = 0;
    if(fabs(ty)/max < pow(10.,-15))ty = 0;
    if(fabs(tz)/max < pow(10.,-15))tz = 0;

    max = fabs(sx);
    if(max < fabs(sy))max = fabs(sy);
    if(max < fabs(sz))max = fabs(sz);

    if(fabs(sx)/max < pow(10.,-15))sx = 0;
    if(fabs(sy)/max < pow(10.,-15))sy = 0;
    if(fabs(sz)/max < pow(10.,-15))sz = 0;
}


void Solid::LoadAccelerationFromFile(FILE *ft) noexcept {
    fscanf(ft,"%lf\t%d",&Mass,&activeGravity);
    fscanf(ft,"%lf\t%lf\t%lf",&Fcx,&Fcy,&Fcz);
    fscanf(ft,"%lf\t%lf\t%lf",&Mcx,&Mcy,&Mcz);
    if(Fcx == 0 && Fcy == 0 && Fcz == 0 && Mcx == 0 && Mcy == 0 && Mcz == 0 && activeGravity == 0)
        Force = 0;
    else
        Force = 1;
}

void Solid::WriteAccelerationFromFile(FILE *ft) const noexcept {
    fprintf(ft,"%e\t%d\n",Mass,activeGravity);
    fprintf(ft,"%e\t%e\t%e\n",Fcx,Fcy,Fcz);
    fprintf(ft,"%e\t%e\t%e\n",Mcx,Mcy,Mcz);
}

void Solid::WriteToFile(FILE *ft) const noexcept {
    fprintf(ft,"%e\t%e\t%e\n",x,y,z);
    fprintf(ft,"%e\t%e\t%e\t%e\n",q0,q1,q2,q3);
    V.WriteToFile(ft,0);
}

void Solid::WriteOutFile(FILE *ft, int mode) const noexcept {
    if(mode == 0){
        fprintf(ft,"%e\t%e\t%e\n",x,y,z);
        fprintf(ft,"%e\t%e\t%e\t%e\n",q0,q1,q2,q3);
    }
    else{
        fwrite(&x, sizeof(double), 1, ft);
        fwrite(&y, sizeof(double), 1, ft);
        fwrite(&z, sizeof(double), 1, ft);

        fwrite(&nx, sizeof(double), 1, ft);
        fwrite(&ny, sizeof(double), 1, ft);
        fwrite(&nz, sizeof(double), 1, ft);

        fwrite(&tx, sizeof(double), 1, ft);
        fwrite(&ty, sizeof(double), 1, ft);
        fwrite(&tz, sizeof(double), 1, ft);

        fwrite(&sx, sizeof(double), 1, ft);
        fwrite(&sy, sizeof(double), 1, ft);
        fwrite(&sz, sizeof(double), 1, ft);
    }
    V.WriteToFile(ft,mode);
}

void Solid::Display() const noexcept {
    printf("%e\t%e\t%e\n",x,y,z);
    printf("%e\t%e\t%e\n",nx,ny,nz);
    printf("%e\t%e\t%e\n",tx,ty,tz);
    printf("%e\t%e\t%e\n",sx,sy,sz);
    V.Display();
}

void Solid::UpDateGravityVelocity(double time, double dt, Gravity& gt) noexcept {
    if(activeGravity == 1){
        if(gt.ngx*gt.G != 0)
            vx += ((Fx)/Mass + gt.ngx*gt.G)*dt;
        if(gt.ngy*gt.G != 0)
            vy += ((Fy)/Mass + gt.ngy*gt.G)*dt;
        if(gt.ngz*gt.G != 0)
            vz += ((Fz)/Mass + gt.ngz*gt.G)*dt;
    }
}


void Solid::UpDateVelocity(double time, double dt, Gravity& gt) noexcept {
    double Mlx,Mly,Mlz,Mn,Mt,Ms,wn,wt,ws;

    if(activeGravity == 1){
        if(Fcx != 0 || gt.ngx*gt.G != 0)
            vx += ((Fcx+Fx)/Mass + gt.ngx*gt.G)*dt;
        else
            vx = V.ValueOfVx(time);
        if(Fcy != 0 || gt.ngy*gt.G != 0)
            vy += ((Fcy+Fy)/Mass + gt.ngy*gt.G)*dt;
        else
            vy = V.ValueOfVy(time);
        if(Fcz != 0 || gt.ngz*gt.G != 0)
            vz += ((Fcz+Fz)/Mass + gt.ngz*gt.G)*dt;
        else
            vz = V.ValueOfVz(time);
    }
    else{
        if(Fcx != 0)
            vx += (Fcx+Fx)/Mass*dt;
        else
            vx = V.ValueOfVx(time);
        if(Fcy != 0)
            vy += (Fcy+Fy)/Mass*dt;
        else
            vy = V.ValueOfVy(time);
        if(Fcz != 0)
            vz += (Fcz+Fz)/Mass*dt;
        else
            vz = V.ValueOfVz(time);
    }

    // Composantes de la force = somme contrainte + reaction
    Mlx = Mcx+Mx;
    Mly = Mcy+My;
    Mlz = Mcz+Mz;
    // Projection local
    Mn = Mlx*nx+Mly*ny+Mlz*nz;
    Mt = Mlx*tx+Mly*ty+Mlz*tz;
    Ms = Mlx*sx+Mly*sy+Mlz*sz;
    // Vitesse angulaire locale
    wn = Mn/In*dt;
    wt = Mt/It*dt;
    ws = Ms/Is*dt;
    // Incrementation dans la base globale
    if(Mcx != 0)
        wx += (wn*nx+wt*tx+ws*sx);
    else
        wx = V.ValueOfWx(time);
    if(Mcy != 0)
        wy += (wn*ny+wt*ty+ws*sy);
    else
        wy = V.ValueOfWy(time);

    if(Mcz != 0)
        wz += (wn*nz+wt*tz+ws*sz);
    else
        wz = V.ValueOfWz(time);
}


void Solid::MoveGravity(double dt, Gravity& gt) noexcept {
    if(activeGravity == 1){
        // Translation
        if(gt.ngx*gt.G != 0)
            x += vx*dt;
        if(gt.ngy*gt.G != 0)
            y += vy*dt;
        if(gt.ngz*gt.G != 0)
            z += vz*dt;
    }
}

void Solid::Move(double dt) noexcept {
    double lx,ly,lz;
    double a,sa,ql0,ql1,ql2,ql3,p0,p1,p2,p3;
    // Translation
    x += vx*dt;
    y += vy*dt;
    z += vz*dt;
    // Rotation
    a = sqrt(wx*wx+wy*wy+wz*wz);
    if(a != 0){
        sa = sin(a*dt/2);
        p0 = cos(a*dt/2);
        p1 = wx/a*sa;
        p2 = wy/a*sa;
        p3 = wz/a*sa;

        ql0 = q0;
        ql1 = q1;
        ql2 = q2;
        ql3 = q3;

        q0 = ql0*p0 - ql1*p1 - ql2*p2 - ql3*p3;
        q1 = ql0*p1 + ql1*p0 - ql2*p3 + ql3*p2;
        q2 = ql0*p2 + ql1*p3 + ql2*p0 - ql3*p1;
        q3 = ql0*p3 - ql1*p2 + ql2*p1 + ql3*p0;

        ComputeBase();

        lx = (x-V.ox);
        ly = (y-V.oy);
        lz = (z-V.oz);

        x = ((1 - 2*p2*p2 - 2*p3*p3)*lx + (2*p1*p2 - 2*p3*p0)*ly     + (2*p1*p3 + 2*p2*p0)*lz)     + V.ox;
        y = ((2*p1*p2 + 2*p3*p0)*lx     + (1 - 2*p1*p1 - 2*p3*p3)*ly + (2*p2*p3 - 2*p1*p0)*lz)     + V.oy;
        z = ((2*p1*p3 - 2*p2*p0)*lx     + (2*p2*p3 + 2*p1*p0)*ly     + (1 - 2*p1*p1 - 2*p2*p2)*lz) + V.oz;
    }
}

void Solid::OnOffGravity(bool OnOff) noexcept {
    if(OnOff){
        activeGravity = 1;
    }
    else{
        activeGravity = 0;
        vx = 0;
        vy = 0;
        vz = 0;
    }
}

void Solid::SetVelocityToZero() noexcept {
    vx = 0;
    vy = 0;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
}

double Solid::Vmax() const noexcept {
    return(V.VMax());
}

double Solid::Wmax() const noexcept {
    return(V.WMax());
}

double Solid::Delay() const noexcept {
    return(V.Delay());
}

void Solid::SetVz(double newA0, double newA1, double newW, double newPhi) noexcept {
    V.Set(Velocity::VelocityType::vz, newA0, newA1, newW, newPhi);
}


void Solid::InitTimeStep() noexcept {
    Fx = 0.;
    Fy = 0.;
    Fz = 0.;
    Mx = 0.;
    My = 0.;
    Mz = 0.;
}

void Solid::SetMemoryPosition() noexcept {
    xMemory = x;
    yMemory = y;
    zMemory = z;
}

void Solid::GetMemoryPosition() noexcept {
    x = xMemory;
    y = yMemory;
    z = zMemory;
}

void Solid::UpDateForce() noexcept {
    if(Fcx == 0 && Fcy == 0 && Fcz == 0 && Mcx == 0 && Mcy == 0 && Mcz == 0 && activeGravity == 0)
        Force = 0;
    else
        Force = 1;
}


