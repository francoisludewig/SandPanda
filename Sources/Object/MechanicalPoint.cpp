//
// Created by ludfr on 18-03-25.
//
#include "../../Includes/Object/MechanicalPoint.h"

#include <cmath>

void MechanicalPoint::move(double dt) {
    double a, sa;
    double p0 = 0, p1 = 0, p2 = 0, p3 = 0;
    double ql0, ql1, ql2, ql3;
    x += vx * dt;
    y += vy * dt;
    z += vz * dt;
    a = sqrt(wx * wx + wy * wy + wz * wz);
    if (a != 0) {
        sa = sin(dt * a / 2);
        p0 = cos(dt * a / 2);
        p1 = wx / a * sa;
        p2 = wy / a * sa;
        p3 = wz / a * sa;

        ql0 = q0;
        ql1 = q1;
        ql2 = q2;
        ql3 = q3;

        if (q0 == 0 && q1 == 0 && q2 == 0 && q3 == 0) {
            q0 = p0;
            q1 = p1;
            q2 = p2;
            q3 = p3;
        } else {
            q0 = ql0 * p0 - ql1 * p1 - ql2 * p2 - ql3 * p3;
            q1 = ql0 * p1 + ql1 * p0 - ql2 * p3 + ql3 * p2;
            q2 = ql0 * p2 + ql1 * p3 + ql2 * p0 - ql3 * p1;
            q3 = ql0 * p3 - ql1 * p2 + ql2 * p1 + ql3 * p0;
        }
    }
}

void MechanicalPoint::resetForceAndMomentum() {
    Fx = 0.;
    Fy = 0.;
    Fz = 0.;
    Mx = 0.;
    My = 0.;
    Mz = 0.;
}
