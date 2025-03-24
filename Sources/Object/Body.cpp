#include <iostream>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include "../../Includes/Object/Body.h"
#include "../../Includes/Object/Sphere.h"

Body::Body() noexcept {
    QuaternionToBase();
    Ng = 0;
    Rmax = 0;
    m = 0;
    Ine_1[0][0] = 0;
    Ine_1[0][1] = 0;
    Ine_1[0][2] = 0;
    Ine_1[1][0] = 0;
    Ine_1[1][1] = 0;
    Ine_1[1][2] = 0;
    Ine_1[2][0] = 0;
    Ine_1[2][1] = 0;
    Ine_1[2][2] = 0;

    cx = 0;
    cy = 0;
    cz = 0;

    sp = 0;
    ActiveRotation = 0;
    /*
    for (int i = 0; i < 250; i++) {
        NumNeighbour[i] = -9;
        type[i] = -1;
        xsi[i].Reset();
        NumFromBody[i] = -9;
        SelfNumFromBody[i] = -9;
    }
    Nneighbour = 0;
    Nneighbour2 = 50;
     */
}

Body::~Body() noexcept {}

void Body::LoadFromFile(FILE *ft) noexcept {
    fscanf(ft, "%d\t%d\t", &sp, &NhollowBall);
    fscanf(ft, "%lf\t%lf\t%lf\t", &x, &y, &z);
    fscanf(ft, "%lf\t%lf\t%lf\t%lf\t", &q0, &q1, &q2, &q3);
    fscanf(ft, "%lf\t%lf\t%lf\t", &vx, &vy, &vz);
    fscanf(ft, "%lf\t%lf\t%lf\n", &wx, &wy, &wz);

    //q Ecriture de la base locale via le quaternion
    QuaternionToBase();
}

void Body::ReadStartStopFile(FILE *ft) noexcept {
    fscanf(ft, "%d\t%d\t", &sp, &NhollowBall);
    fscanf(ft, "%lf\t%lf\t%lf\t", &x, &y, &z);
    fscanf(ft, "%lf\t%lf\t%lf\t%lf\t", &q0, &q1, &q2, &q3);
    fscanf(ft, "%lf\t%lf\t%lf\t", &vx, &vy, &vz);
    fscanf(ft, "%lf\t%lf\t%lf\n", &wx, &wy, &wz);
    elongationManager.readFromFile(ft);
    /*
    fscanf(ft, "%d\n", &Nneighbour2);
    Nneighbour2 += 50;
    for (int i = 50; i < Nneighbour2; i++)
        fscanf(ft, "%d\t%d\t%d\t%d\t%lf\t%lf\t%lf\n", &NumNeighbour[i], &type[i], &SelfNumFromBody[i], &NumFromBody[i],
               &xsi[i].x, &xsi[i].y, &xsi[i].z);
               */
    //q Ecriture de la base locale via le quaternion
    QuaternionToBase();
}

void Body::WriteToFile(FILE *ft) const noexcept {
    fprintf(ft, "%d\t%d\t", sp, NhollowBall);
    fprintf(ft, "%.15f\t%.15f\t%.15f\t", x, y, z);
    fprintf(ft, "%e\t%e\t%e\t%e\t", q0, q1, q2, q3);
    fprintf(ft, "%e\t%e\t%e\t", vx, vy, vz);
    fprintf(ft, "%e\t%e\t%e\n", wx, wy, wz);
    elongationManager.writeToFile(ft);
    /*
    fprintf(ft, "%d\n", Nneighbour);
    for (int i = 0; i < Nneighbour; i++)
        fprintf(ft, "%d\t%d\t%d\t%d\t%e\t%e\t%e\n", NumNeighbour[i], type[i], SelfNumFromBody[i], NumFromBody[i],
                xsi[i].x, xsi[i].y, xsi[i].z);
                */
}

void Body::WriteOutFile(FILE *ft, int mode) const noexcept {
    if (mode == 0) {
        fprintf(ft, "%d\t", sp);
        fprintf(ft, "%e\t%e\t%e\t", x, y, z);
        fprintf(ft, "%e\t%e\t%e\t%e\t", q0, q1, q2, q3);
        fprintf(ft, "%e\t%e\t%e\t", vx, vy, vz);
        fprintf(ft, "%e\t%e\t%e\n", wx, wy, wz);
    } else {
        fwrite(&sp, sizeof(int), 1, ft);
        fwrite(&x, sizeof(double), 1, ft);
        fwrite(&y, sizeof(double), 1, ft);
        fwrite(&z, sizeof(double), 1, ft);

        fwrite(&q0, sizeof(double), 1, ft);
        fwrite(&q1, sizeof(double), 1, ft);
        fwrite(&q2, sizeof(double), 1, ft);
        fwrite(&q3, sizeof(double), 1, ft);

        fwrite(&vx, sizeof(double), 1, ft);
        fwrite(&vy, sizeof(double), 1, ft);
        fwrite(&vz, sizeof(double), 1, ft);

        fwrite(&wx, sizeof(double), 1, ft);
        fwrite(&wy, sizeof(double), 1, ft);
        fwrite(&wz, sizeof(double), 1, ft);

    }
}

void Body::UpDateVelocity(double dt, Gravity &g) noexcept {
    double Mn, Mt, Ms, wn, wt, ws;
    Fx += m * g.ngx * g.G;
    Fy += m * g.ngy * g.G;
    Fz += m * g.ngz * g.G;

    vx += Fx / m * dt;
    vy += Fy / m * dt;
    vz += Fz / m * dt;
    if (ActiveRotation == 0) {
        // Projection local
        Mn = Mx * nx + My * ny + Mz * nz;
        Mt = Mx * tx + My * ty + Mz * tz;
        Ms = Mx * sx + My * sy + Mz * sz;

        // Vitesse angulaire locale
        wn = (Ine_1[0][0] * Mn + Ine_1[0][1] * Mt + Ine_1[0][2] * Ms) * dt;
        wt = (Ine_1[1][0] * Mn + Ine_1[1][1] * Mt + Ine_1[1][2] * Ms) * dt;
        ws = (Ine_1[2][0] * Mn + Ine_1[2][1] * Mt + Ine_1[2][2] * Ms) * dt;

        // Incrementation dans la base globale
        wx += (wn * nx + wt * tx + ws * sx);
        wy += (wn * ny + wt * ty + ws * sy);
        wz += (wn * nz + wt * tz + ws * sz);
    }
}

inline void Body::QuaternionToBase() noexcept {
    nx = 1 - 2 * q2 * q2 - 2 * q3 * q3;
    ny = 2 * q1 * q2 + 2 * q3 * q0;
    nz = 2 * q1 * q3 - 2 * q2 * q0;

    tx = 2 * q1 * q2 - 2 * q3 * q0;
    ty = 1 - 2 * q1 * q1 - 2 * q3 * q3;
    tz = 2 * q2 * q3 + 2 * q1 * q0;

    sx = 2 * q1 * q3 + 2 * q2 * q0;
    sy = 2 * q2 * q3 - 2 * q1 * q0;
    sz = 1 - 2 * q1 * q1 - 2 * q2 * q2;
}

void Body::move(double dt) noexcept {
    MechanicalPoint::move(dt);
    // Ecriture de la nouvelle base locale via le quaternion
    QuaternionToBase();
}

void Body::UpDateLinkedSphere(vector<Sphere> &sph) noexcept {
    UpDateLinkedSphereTp();
    sph[numl].x = x;
    sph[numl].y = y;
    sph[numl].z = z;
}

void Body::UpDateLinkedSphereTp() noexcept {
    for (int j = 0; j < Ng; j++) {
        xg[j] = x + nx * xl[j] + tx * yl[j] + sx * zl[j];
        yg[j] = y + ny * xl[j] + ty * yl[j] + sy * zl[j];
        zg[j] = z + nz * xl[j] + tz * yl[j] + sz * zl[j];
    }
}

void Body::RandomVelocity(double V, double W) noexcept {
    double beta, alpha, rdm, Norme;
    beta = 2 * M_PI * (double) (rand() % RAND_MAX) / RAND_MAX;
    rdm = (double) (rand() % RAND_MAX) / RAND_MAX;
    alpha = acos(1 - 2 * rdm);
    vz = cos(alpha);
    vx = cos(beta) * sin(alpha);
    vy = sin(beta) * sin(alpha);
    beta = 2 * M_PI * (double) (rand() % RAND_MAX) / RAND_MAX;
    rdm = (double) (rand() % RAND_MAX) / RAND_MAX;
    alpha = acos(1 - 2 * rdm);
    wz = cos(alpha);
    wx = cos(beta) * sin(alpha);
    wy = sin(beta) * sin(alpha);
    Norme = sqrt(vx * vx + vy * vy + vz * vz);
    vx = vx / Norme * V;
    vy = vy / Norme * V;
    vz = vz / Norme * V;
    Norme = sqrt(wx * wx + wy * wy + wz * wz);
    wx = wx / Norme * W;
    wy = wy / Norme * W;
    wz = wz / Norme * W;
}

void Body::UploadSpecies(vector<BodySpecie> &bdsp, vector<Sphere> &sph, int &Nsph, int numero) noexcept {
    Ng = bdsp[sp].Ng;

    for (int j = 0; j < Ng; j++) {
        xl.push_back(bdsp[sp].xl[j]);
        yl.push_back(bdsp[sp].yl[j]);
        zl.push_back(bdsp[sp].zl[j]);
        r.push_back(bdsp[sp].rl[j]);
        if (Rmax < r[j])Rmax = r[j];
        xg.push_back(x + nx * bdsp[sp].xl[j] + tx * bdsp[sp].yl[j] + sx * bdsp[sp].zl[j]);
        yg.push_back(y + ny * bdsp[sp].xl[j] + ty * bdsp[sp].yl[j] + sy * bdsp[sp].zl[j]);
        zg.push_back(z + nz * bdsp[sp].xl[j] + tz * bdsp[sp].yl[j] + sz * bdsp[sp].zl[j]);
    }

    Sphere *sphl = new Sphere();
    sphl->x = x;
    sphl->y = y;
    sphl->z = z;
    sphl->r = bdsp[sp].FeretMax / 2.;
    sphl->num = Nsph;
    sphl->bodies = numero;
    sphl->NhollowBall = NhollowBall;
    numl = Nsph;
    sph.push_back(*sphl);
    Nsph++;
    delete sphl;

    //Correction de la densite 7700 -> 1000
    //printf("m = %e\n",bdsp[sp].m);
    m = bdsp[sp].m;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Ine_1[i][j] = bdsp[sp].Ine_1[i][j];
        }
    }
}

void Body::InitXsi() noexcept {
    elongationManager.InitXsi();
/*
    for (int i = 50; i < Nneighbour2; i++) {
        xsi[i - 50] = xsi[i];
        NumNeighbour[i - 50] = NumNeighbour[i];
        type[i - 50] = type[i];
        NumFromBody[i - 50] = NumFromBody[i];
        SelfNumFromBody[i - 50] = SelfNumFromBody[i];
        xsi[i].Reset();
        NumNeighbour[i] = -9;
        type[i] = -1;
        NumFromBody[i] = -9;
        SelfNumFromBody[i] = -9;
    }
    Nneighbour = Nneighbour2 - 50;
    Nneighbour2 = 50;
    */
}
/*
void Body::AddXsi(Elongation e, int n, int t, int selfn, int nob) noexcept {
    xsi[Nneighbour2] = e;
    NumNeighbour[Nneighbour2] = n;
    type[Nneighbour2] = t;
    NumFromBody[Nneighbour2] = nob;
    SelfNumFromBody[Nneighbour2] = selfn;
    Nneighbour2++;
    if (Nneighbour2 > 250) {
        printf("Alert number of contact per particle is over 250\n");
    }
}

Elongation Body::FoundIt(int n, int t, int selfn, int nob) const noexcept {
    for (int i = 0; i < Nneighbour; i++) {
        if (NumNeighbour[i] == n && type[i] == t && SelfNumFromBody[i] == selfn && NumFromBody[i] == nob)
            return xsi[i];
    }
    Elongation e;
    e.Reset();
    return (e);
}
*/

void Body::AddXsi(Elongation& e, uint64_t contactIdentifier) noexcept {
    elongationManager.AddXsi(e, contactIdentifier);
}
Elongation Body::FoundIt(uint64_t contactIdentifier) const noexcept {
    return elongationManager.FoundIt(contactIdentifier);
}

int Body::NumberOfSphere() const noexcept {
    return Ng;
}

int Body::Num() const noexcept {
    return numl;
}

double Body::GetRmax() const noexcept {
    return Rmax;
}

void Body::SetActiveRotation(int na) noexcept {
    if (na == 0 || na == 1) {
        ActiveRotation = na;
    }
}
