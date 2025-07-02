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
#pragma omp parallel shared(cell, Ntp, name, isMonitoringActivated) firstprivate(ct, cellBounds, Nct)
    {
        const double dt = solids->configuration.dt;
        ct = new Contact[18 * solids->spheres.size() + 75 * solids->bodies.size()];
        auto nthrd = omp_get_thread_num();
        auto thrd_start = omp_get_num_threads();
        int xstep = (cellBounds.EndX() - cellBounds.StartX()) / thrd_start;
        CellBounds thread_cellBounds;
        if (nthrd != thrd_start - 1) {
            thread_cellBounds = CellBounds(cellBounds.MinX(), cellBounds.MinY(), cellBounds.MinZ(),
                                           cellBounds.MaxX(), cellBounds.MaxY(), cellBounds.MaxZ(),
                                           cellBounds.StartX() + (nthrd) * xstep, cellBounds.StartY(),
                                           cellBounds.StartZ(),
                                           cellBounds.StartX() + (nthrd + 1) * xstep, cellBounds.EndY(),
                                           cellBounds.EndZ(),
                                           cellBounds.Lx(), cellBounds.Ly(), cellBounds.Lz(),
                                           cellBounds.XMin(), cellBounds.YMin(), cellBounds.ZMin());
        } else {
            thread_cellBounds = CellBounds(cellBounds.MinX(), cellBounds.MinY(), cellBounds.MinZ(),
                                           cellBounds.MaxX(), cellBounds.MaxY(), cellBounds.MaxZ(),
                                           cellBounds.StartX() + (nthrd) * xstep, cellBounds.StartY(),
                                           cellBounds.StartZ(),
                                           cellBounds.EndX(), cellBounds.EndY(), cellBounds.EndZ(),
                                           cellBounds.Lx(), cellBounds.Ly(), cellBounds.Lz(),
                                           cellBounds.XMin(), cellBounds.YMin(), cellBounds.ZMin());
        }
        printf("Thread %d/%d - cellbounds X : %d - %d [Step = %d]\n", nthrd+1, thrd_start, thread_cellBounds.StartX(),
               thread_cellBounds.EndX(), xstep);
        // Sequential Version
        printf("Evolution\n");
        do {
#pragma omp barrier
#pragma omp single
            {
                //printf("Start loop by : %d\n", nthrd);
                solids->configuration.TIME += dt;
                // Position anticipation
                Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows,
                                    solids->configuration.TIME, dt / 2, solids->spheres, solids->gravity);
                solids->configuration.mas->Move(dt / 2);

                Move::moveSphere(solids->spheres, dt / 2);
                Move::upDateHollowBall(solids->hollowBalls, dt);

                solids->gravity.Move(solids->configuration.TIME, dt / 2);
                //Doublon pour test
                Move::moveBodies(solids->bodies, dt / 2, solids->spheres);
                PeriodicityPL(solids->spheres, solids->plans);

                // Linked Cells
                LinkedCellFiller::Fill(solids->spheres, solids->configuration, cell);
                // Initialization for the time step
                ComputeForce::InitForTimeStep(Nct, solids->spheres, solids->bodies, ct, solids->plans, solids->disks,
                                              solids->cones, solids->elbows);

            }
            // Contact Detection
            Nct = 0;
            // Verison sequentiel normale
            ContactDetection::sphContact(thread_cellBounds, ct, Nct, cell);
            //printf("%d contacts detected by %d\n", Nct, nthrd);
#pragma omp single
            {
                //printf("ContactDetection by : %d\n", nthrd);

                ContactDetection::sphContainer(solids->spheres, solids->plans, solids->disks, solids->cones,
                                               solids->elbows, solids->hollowBalls, Nct, ct, cell, solidCells,
                                               solids->configuration.Rmax);
            }
            // Computing Force
            ComputeForce::Compute(ct, Nct, solids->configuration);
            //printf("ComputeForce (%d) by %d\n", Nct, nthrd);

#pragma omp critical
            {
                ComputeForce::SumForceAndMomentum(ct, Nct);
                //printf("SumForceAndMomentum (%d) by %d\n", Nct, nthrd);
            }
#pragma omp barrier
#pragma omp single
            {
                //printf("End of loop by : %d\n", nthrd);
                Move::UpDateForceContainer(solids->spheres, solids->plans, solids->disks, solids->cones,
                                           solids->configuration.TIME, dt, solids->gravity);
                solids->configuration.mas->getForces();
                // Update Velocities
                Move::upDateVelocitySphere(solids->spheres, solids->gravity, dt);
                Move::upDateVelocityBodies(solids->bodies, solids->gravity, dt, solids->spheres);
                Move::upDateVelocityContainer(solids->plans, solids->disks, solids->cones, solids->elbows,
                                              solids->configuration.TIME, dt, solids->gravity);
                solids->configuration.mas->UpDateVelocity(dt);

                // Move
                Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows,
                                    solids->configuration.TIME, dt / 2, solids->spheres, solids->gravity);
                solids->configuration.mas->Move(dt / 2);
                Move::moveSphere(solids->spheres, dt / 2);
                Move::moveBodies(solids->bodies, dt / 2, solids->spheres);
                Move::upDateHollowBall(solids->hollowBalls, dt);

                solids->gravity.Move(solids->configuration.TIME, dt / 2);


                PeriodicityPL(solids->spheres, solids->plans);

                // Record data
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

                        //writeOutData(name, Ntp, &gf, &dat);
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
            }
        } while (solids->configuration.TIME <= solids->configuration.Total - solids->configuration.dt * 0.99);
        //printf("TIME = %.20lf > %.20lf = Total-dt\n",dat.TIME,dat.Total-dat.dt*0.9);
        printf("\n");

        delete[] ct;
    }
    return (Ntp);
}