#include "../../Includes/Dynamic/Compaction.h"

#include "../../Includes/Solids/Plan.h"
#include "../../Includes/Solids/PlanR.h"
#include "../../Includes/Solids/Cone.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/Solids/Body.h"
#include "../../Includes/Repository/SimulationData.h"
#include "../../Includes/Repository/ReadWrite.h"
#include "../../Includes/ComputingForce.h"
#include "../../Includes/Configuration/Configuration.h"
#include "../../Includes/Configuration/Monitoring.h"
#include "../../Includes/Dynamic/Evolution.h"

void Compaction::Secousse(vector<Plan> &pl, vector<PlanR> &plr, vector<Cone> &co, double Gamma, double f,
                          Configuration &dat) noexcept {
    const double w = 2 * M_PI * f;
    const double A = Gamma * 9.81 / w / w;

    printf("A = %e, w = %e , Aw = %e\n", A, w, A * w);

    for (auto &plan: pl) {
        plan.SetMemoryPosition();
        plan.SetVz(0, A * w, -w,M_PI);
    }

    for (auto &disk: plr) {
        disk.SetMemoryPosition();
        disk.SetVz(0, A * w, -w,M_PI);
    }

    for (auto &cone: co) {
        cone.SetMemoryPosition();
        cone.SetVz(0, A * w, -w,M_PI);
    }

    dat.Total += 1. / f;
    int Ntsp = static_cast<int>((1. / f) / (dat.dt)) + 1;
    dat.dt = (1. / f) / static_cast<double>(Ntsp);
    printf("dt = %e\n", dat.dt);
}


void Compaction::SecousseWithoutConvection(vector<Plan> &pl, vector<PlanR> &plr, vector<Cone> &co, double Gamma,
                                           double f, Configuration &dat) noexcept {
    const double w = 2 * M_PI * f;
    const double A = Gamma * 9.81 / w / w;

    printf("A = %e, w = %e , Aw = %e\n", A, w, A * w);

    for (auto &plan: pl)
        plan.SetVz(0, A * w, -w,M_PI);

    for (auto &disk: plr)
        disk.SetVz(0, A * w, -w,M_PI);

    for (auto &cone: co)
        cone.SetVz(0, A * w, -w,M_PI);

    const double delay = (acos(1.0 / Gamma) - M_PI) / (-w);
    dat.Total += delay;
    int Ntsp = static_cast<int>(delay / (dat.dt)) + 1;
    dat.dt = delay / static_cast<double>(Ntsp);
    printf("dt = %e\n", dat.dt);
}

void Compaction::FreeFall(std::vector<Plan> &pl, std::vector<PlanR> &plr, std::vector<Cone> &co, double Gamma, double f,
                          Configuration &dat) noexcept {
    for (auto &plan: pl) {
        plan.OnOffGravity(true);
        plan.OnOffNoResponse(true);
    }
    for (auto &disk: plr) {
        disk.OnOffGravity(true);
        disk.OnOffNoResponse(true);
    }
    for (auto &cone: co) {
        cone.OnOffGravity(true);
        cone.OnOffNoResponse(true);
    }
    const double w = 2 * M_PI * f;
    const double A = Gamma * 9.81 / w / w;
    const double delay = (acos(1.0 / Gamma) - M_PI) / (-w);
    const double pt = A * cos(w * delay + M_PI);
    const double vt = -A * w * sin(w * delay + M_PI);

    const double a = -9.81 / 2;
    const double b = vt;
    const double c = pt + A;
    const double radian = b * b - 4 * a * c;
    const double tm = (-b - sqrt(radian)) / (2 * a);
    const double freeFallDelay = tm + delay;
    printf("FreefallDelay(tap) = %e\n", tm);
    printf("All delay(tap) = %e\n", freeFallDelay);
    dat.Total += freeFallDelay;
}

void Compaction::Relaxation(std::vector<Plan> &pl, std::vector<PlanR> &plr, std::vector<Cone> &co,
                            Configuration &dat) noexcept {
    for (auto &plan: pl) {
        plan.SetVz(0, 0, 0, 0);
        plan.OnOffGravity(false);
        plan.GetMemoryPosition();
    }

    for (auto &disk: plr) {
        disk.SetVz(0, 0, 0, 0);
        disk.OnOffGravity(false);
        disk.GetMemoryPosition();
    }

    for (auto &cone: co) {
        cone.SetVz(0, 0, 0, 0);
        cone.OnOffGravity(false);
        cone.GetMemoryPosition();
    }

    dat.Total += 0.5;
}

int Compaction::Run(std::shared_ptr<SimulationData> &solids, std::vector<Sphere *> &cell, int &Ntp, char *name,
                    int ntpi, int ntpf, double Gamma, double f, const CellBounds &cellBounds, bool
                    isMonitoringActivated, bool withoutConvection) noexcept {
    Evolution evolution(solids, cellBounds);
    solids->configuration.record = false;
    for (int nt = ntpi; nt <= ntpf; nt++) {
        if (!withoutConvection) {
            //Secousse
            Secousse(solids->plans, solids->disks, solids->cones, Gamma, f, solids->configuration);
            printf("Total = %e\n", solids->configuration.Total);
            Ntp = evolution.Evolve(cell, Ntp, name, false);
            // Relaxation
            Relaxation(solids->plans, solids->disks, solids->cones, solids->configuration);
            printf("Total = %e\n", solids->configuration.Total);
            Ntp = evolution.Evolve(cell, Ntp, name, false);
        } else {
            //Secousse
            SecousseWithoutConvection(solids->plans, solids->disks, solids->cones, Gamma, f, solids->configuration);
            printf("Total = %e\n", solids->configuration.Total);
            Ntp = evolution.Evolve(cell, Ntp, name, false);
            //Chute libre
            FreeFall(solids->plans, solids->disks, solids->cones, Gamma, f, solids->configuration);
            printf("Total = %e\n", solids->configuration.Total);
            Ntp = evolution.Evolve(cell, Ntp, name, false);
            // Relaxation
            Relaxation(solids->plans, solids->disks, solids->cones, solids->configuration);
            printf("Total = %e\n", solids->configuration.Total);
            Ntp = evolution.Evolve(cell, Ntp, name, false);
        }

        if(solids->configuration.record == 0){
            // Enregistrement
            ReadWrite::writeStartStopContainer(name,solids->plans,solids->disks,solids->cones,solids->elbows);
            ReadWrite::writeStartStopSphere(name,solids->spheres);
            ReadWrite::writeStartStopBodies(name,solids->bodies,solids->spheres);
            ReadWrite::writeStartStopData(name, solids->gravity, solids->configuration);
            ReadWrite::writeStartStopHollowBall(name, solids->hollowBalls);

            ReadWrite::writeOutContainer(name,nt,solids->plans,solids->disks,solids->cones,solids->elbows,solids->configuration.outMode);
            ReadWrite::writeOutSphere(name,nt,solids->spheres,solids->configuration.outMode);
            ReadWrite::writeOutBodies(name,nt,solids->bodies,solids->configuration.outMode);
            //writeOutData(name, nt, &gf, &dat);
            ReadWrite::writeOutHollowBall(name, Ntp, solids->hollowBalls);
            printf("Ntp = %d\n",nt);
            Ntp++;
            if (isMonitoringActivated) {
                Monitoring::getInstance().metrics(nt, ntpf);
            }
        }
    }
    return Ntp;
}

int Compaction::OneTap(const std::shared_ptr<SimulationData> &solids, std::vector<Sphere *> &cell, int &Ntp, char *name, const double Gamma,
                        const double f, const CellBounds &cellBounds) noexcept {
    solids->configuration.dts = 1.0/f/100;
    solids->configuration.Total = 0.0;
    solids->configuration.t0 = 0.0;
    solids->configuration.record = true;
    Evolution evolution(solids, cellBounds);
    //Secousse
    Secousse(solids->plans, solids->disks, solids->cones, Gamma, f, solids->configuration);
    printf("Total = %e\n", solids->configuration.Total);
    evolution.Evolve(cell, Ntp, name, false);
    // Relaxation
    Relaxation(solids->plans, solids->disks, solids->cones, solids->configuration);
    printf("Total = %e\n", solids->configuration.Total);
    evolution.Evolve(cell, Ntp, name, false);

    return 0;
}
