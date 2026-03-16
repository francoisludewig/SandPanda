#include <ctime>

#include "../../Includes/Dynamic/Evolution.h"

#include <omp.h>

#include "../../Includes/ComputingForce.h"
#include "../../Includes/Configuration/Configuration.h"
#include "../../Includes/Dynamic/Move.h"
#include "../../Includes/Repository/ReadWrite.h"
#include "../../Includes/Contact/ContactDetection.h"
#include "../../Includes/Dynamic/Periodicity.h"
#include "../../Includes/Configuration/Gravity.h"
#include "../../Includes/Configuration/Monitoring.h"
#include "../../Includes/Solids/Sphere.h"
#include "../../Includes/LinkedCells/LinkedCellFiller.h"

int Evolution::Evolve(std::vector<Sphere *> &cell, int &Ntp, char *name, const bool isMonitoringActivated) noexcept {
    const double dt = solids->configuration.dt;
    ct = new Contact[18 * solids->spheres.size() + 75 * solids->bodies.size()];

    printf("Evolution\n");
    do {
        solids->configuration.TIME += dt;

        // === Phase 1: Position anticipation (containers are few, keep sequential) ===
        Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows,
                            solids->configuration.TIME, dt / 2, solids->spheres, solids->gravity);
        solids->configuration.mas->Move(dt / 2);

        // Spheres and bodies: parallelized internally via OpenMP
        Move::moveSphere(solids->spheres, dt / 2);
        Move::upDateHollowBall(solids->hollowBalls, dt);

        solids->gravity.Move(solids->configuration.TIME, dt / 2);
        Move::moveBodies(solids->bodies, dt / 2, solids->spheres);
        PeriodicityPL(solids->spheres, solids->plans);

        // === Phase 2: Linked Cells (inherently sequential due to linked-list construction) ===
        LinkedCellFiller::Fill(solids->spheres, solids->configuration, cell);

        // === Phase 3: Initialization (parallelized internally) ===
        ComputeForce::InitForTimeStep(Nct, solids->spheres, solids->bodies, ct, solids->plans, solids->disks,
                                      solids->cones, solids->elbows);

        // === Phase 4: Contact Detection (parallelized internally via thread-local buffers) ===
        Nct = 0;
        ContactDetection::sphContact(cellBounds, ct, Nct, cell);
        ContactDetection::sphContainer(solids->spheres, solids->plans, solids->disks, solids->cones,
                                       solids->elbows, solids->hollowBalls, Nct, ct, cell, solidCells,
                                       solids->configuration.Rmax);

        // === Phase 5: Force computation (parallelized internally) ===
        ComputeForce::Compute(ct, Nct, solids->configuration);

        // === Phase 6: Sum forces (sequential - accumulates onto shared objects) ===
        ComputeForce::SumForceAndMomentum(ct, Nct);

        // === Phase 7: Update forces and velocities ===
        Move::UpDateForceContainer(solids->spheres, solids->plans, solids->disks, solids->cones,
                                   solids->configuration.TIME, dt, solids->gravity);
        solids->configuration.mas->getForces();

        // Velocity updates: parallelized internally
        Move::upDateVelocitySphere(solids->spheres, solids->gravity, dt);
        Move::upDateVelocityBodies(solids->bodies, solids->gravity, dt, solids->spheres);
        Move::upDateVelocityContainer(solids->plans, solids->disks, solids->cones, solids->elbows,
                                      solids->configuration.TIME, dt, solids->gravity);
        solids->configuration.mas->UpDateVelocity(dt);

        // === Phase 8: Final position update ===
        Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows,
                            solids->configuration.TIME, dt / 2, solids->spheres, solids->gravity);
        solids->configuration.mas->Move(dt / 2);
        Move::moveSphere(solids->spheres, dt / 2);
        Move::moveBodies(solids->bodies, dt / 2, solids->spheres);
        Move::upDateHollowBall(solids->hollowBalls, dt);

        solids->gravity.Move(solids->configuration.TIME, dt / 2);

        PeriodicityPL(solids->spheres, solids->plans);

        // === Phase 9: Record data ===
        if (solids->configuration.record) {
            if (fabs((solids->configuration.TIME - solids->configuration.t0) - Ntp * (solids->configuration.
                         dts)) < solids->configuration.dt * 0.99 && (
                    solids->configuration.TIME - solids->configuration.t0 > 0.)) {
                ReadWrite::writeStartStopContainer(name, solids->plans, solids->disks, solids->cones,
                                                   solids->elbows);
                ReadWrite::writeStartStopSphere(name, solids->spheres);
                ReadWrite::writeStartStopBodies(name, solids->bodies, solids->spheres);
                ReadWrite::writeStartStopData(name, solids->gravity, solids->configuration);
                ReadWrite::writeStartStopHollowBall(name, solids->hollowBalls);

                ReadWrite::writeOutContainer(name, Ntp, solids->plans, solids->disks, solids->cones,
                                             solids->elbows, solids->configuration.outMode);
                ReadWrite::writeOutSphere(name, Ntp, solids->spheres, solids->configuration.outMode);
                ReadWrite::writeOutBodies(name, Ntp, solids->bodies, solids->configuration.outMode);
                ReadWrite::writeOutHollowBall(name, Ntp, solids->hollowBalls);

                if (solids->configuration.outContact == 1 || solids->configuration.outContact > 2)
                    ReadWrite::writeOutContact(name, Ntp, Nct, ct, solids->configuration);
                if (solids->configuration.outContact >= 2)
                    ReadWrite::writeOutContactDetails(name, Ntp, Nct, ct, solids->configuration);
                printf("Save File %d\t\ttime = %e\r", Ntp, solids->configuration.TIME);
                fflush(stdout);
                Ntp++;
                if (isMonitoringActivated) {
                    Monitoring::getInstance().metrics(solids->configuration.TIME, solids->configuration.Total);
                }
            }
        }
    } while (solids->configuration.TIME <= solids->configuration.Total - solids->configuration.dt * 0.99);
    printf("\n");
    delete[] ct;
    return Ntp;
}
