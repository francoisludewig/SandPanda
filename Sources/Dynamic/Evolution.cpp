#include <ctime>
#include <algorithm>
#include <chrono>

#include "../../Includes/Dynamic/Evolution.h"

#ifdef _OPENMP
#include <omp.h>
#endif


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

namespace {
    inline int ThreadNum() noexcept {
#ifdef _OPENMP
        return omp_get_thread_num();
#else
        return 0;
#endif
    }

    inline int ThreadCount() noexcept {
#ifdef _OPENMP
        return omp_get_num_threads();
#else
        return 1;
#endif
    }

    /* Tampon de contacts propre a un thread. "previous" memorise le nombre de contacts
     * utilises au pas precedent afin de les reinitialiser exactement comme le faisait
     * ComputeForce::InitForTimeStep sur le tableau unique. */
    /* Chronometrage optionnel des phases (compiler avec -DSANDPANDA_PHASE_TIMING) */
    struct PhaseTimer {
#ifdef SANDPANDA_PHASE_TIMING
        static constexpr int N = 5;
        double total[N]{};
        std::chrono::steady_clock::time_point last{std::chrono::steady_clock::now()};
        void Mark(const int phase, const int thread) noexcept {
            if (thread != 0) return;
            const auto now = std::chrono::steady_clock::now();
            total[phase] += std::chrono::duration<double>(now - last).count();
            last = now;
        }
        void Print() const noexcept {
            printf("\nPhases (s) : integration+cellules %.2f | detection %.2f | forces %.2f | sommation %.2f | vitesses+sauvegarde %.2f\n",
                   total[0], total[1], total[2], total[3], total[4]);
        }
#else
        void Mark(int, int) noexcept {}
        void Print() const noexcept {}
#endif
    };

    struct ContactBuffer {
        Contact *data{nullptr};
        int count{0};
        int previous{0};
        // Contributions compactes (forces et elongations) produites par ce thread,
        // rangees par thread proprietaire de l'objet cible
        std::vector<std::vector<ComputeForce::ForceContribution>> forces;
        std::vector<std::vector<ComputeForce::XsiContribution>> xsis;
    };
}

/* Construction de la liste ordonnee des blocs de detection.
 * L'ordre des blocs reproduit exactement l'ordre de la version sequentielle :
 *   1. contacts sphere-sphere, plan X par plan X (i croissant)
 *   2. plans, 3. disques, 4. cones (decoupes en tranches de cellules), 5. coudes, 6. boules creuses */
void Evolution::BuildDetectionTasks() noexcept {
    tasks.clear();
    for (int i = cellBounds.StartX(); i < cellBounds.EndX(); i++)
        tasks.push_back({DetectionTask::Kind::SpherePlaneX, i, 0, 0});
    if (!solids->plans.empty())
        tasks.push_back({DetectionTask::Kind::Plans, 0, 0, 0});
    for (int d = 0; d < static_cast<int>(solids->disks.size()); d++)
        tasks.push_back({DetectionTask::Kind::Disk, d, 0, 0});
    for (int c = 0; c < static_cast<int>(solids->cones.size()); c++) {
        const int nCells = static_cast<int>(solidCells.ConeCells(solids->cones[c].Numero()).Size());
        for (int j = 0; j < nCells; j += coneCellsPerTask)
            tasks.push_back({DetectionTask::Kind::ConeRange, c, j, std::min(j + coneCellsPerTask, nCells)});
    }
    for (int e = 0; e < static_cast<int>(solids->elbows.size()); e++)
        tasks.push_back({DetectionTask::Kind::Elbow, e, 0, 0});
    if (!solids->hollowBalls.empty())
        tasks.push_back({DetectionTask::Kind::HollowBalls, 0, 0, 0});
}

void Evolution::RunDetectionTask(const DetectionTask &task, Contact *ct, int &Nct,
                                 const std::vector<Sphere *> &cell) noexcept {
    switch (task.kind) {
        case DetectionTask::Kind::SpherePlaneX:
            ContactDetection::sphContactPlane(cellBounds, task.index, ct, Nct, cell);
            break;
        case DetectionTask::Kind::Plans: {
            // Les plans periodiques utilisent des listes partagees : un seul bloc
            auto &mutableCell = const_cast<std::vector<Sphere *> &>(cell);
            ContactDetection::sphPlanContact(Nct, solids->spheres, solids->plans, ct, mutableCell, solidCells,
                                             solids->configuration.Rmax);
            break;
        }
        case DetectionTask::Kind::Disk:
            ContactDetection::sphPlanRContactOne(Nct, solids->disks[task.index], ct, cell, solidCells);
            break;
        case DetectionTask::Kind::ConeRange:
            ContactDetection::sphConeContactRange(Nct, solids->cones[task.index], ct, cell, solidCells,
                                                  task.begin, task.end);
            break;
        case DetectionTask::Kind::Elbow:
            ContactDetection::sphElbowContactOne(Nct, solids->elbows[task.index], ct, cell, solidCells);
            break;
        case DetectionTask::Kind::HollowBalls:
            ContactDetection::sphHollowBallContact(Nct, solids->hollowBalls, ct);
            break;
    }
}

/*
 * Boucle temporelle.
 *
 * Strategie OpenMP : une seule region parallele pour toute la simulation.
 *  - Integration (positions, vitesses, initialisation) des spheres : "omp for", chaque
 *    sphere est independante, le resultat ne depend pas du nombre de threads.
 *  - Detection : les blocs ordonnes (BuildDetectionTasks) sont distribues dynamiquement ;
 *    chaque thread ecrit dans son propre tampon et on note, pour chaque bloc, ou se trouvent
 *    ses contacts.
 *  - Calcul des forces : independant pour chaque contact, distribue par bloc.
 *  - Sommation des forces et des elongations : sequentielle, bloc apres bloc, dans l'ordre
 *    de la liste, c'est-a-dire exactement dans l'ordre de la version sequentielle.
 * Les resultats sont donc identiques, bit a bit, quel que soit le nombre de threads.
 */
int Evolution::Evolve(std::vector<Sphere *> &cell, int &Ntp, char *name, const bool isMonitoringActivated) noexcept {
    const std::size_t contactCapacity = 18 * solids->spheres.size() + 75 * solids->bodies.size() + 1;
    BuildDetectionTasks();
    std::vector<TaskOutput> taskOutputs(tasks.size());
    std::vector<ContactBuffer> buffers;
    std::vector<Contact> contactsForOutput;
    PhaseTimer timer;
    // Decision de poursuivre la boucle, prise par un seul thread (dans le dernier "single" du pas)
    // et lue par tous apres sa barriere implicite : TIME ne peut pas etre modifie entre-temps.
    bool keepGoing = true;
    const int nTasks = static_cast<int>(tasks.size());
    auto &spheres = solids->spheres;
    const int nSpheres = static_cast<int>(spheres.size());
    std::vector<int> sphereOwner(nSpheres, 0);

#pragma omp parallel shared(cell, Ntp, name, isMonitoringActivated, taskOutputs, buffers, contactsForOutput, timer, sphereOwner, keepGoing)
    {
        const double dt = solids->configuration.dt;
        const int thread = ThreadNum();
        const int nThreads = ThreadCount();
        // Partition fixe des spheres : le thread "thread" integre et somme les forces des
        // spheres [sphereBegin, sphereEnd[ (meme partition a chaque phase -> donnees en cache)
        const int sphereBegin = static_cast<int>(static_cast<long long>(nSpheres) * thread / nThreads);
        const int sphereEnd = static_cast<int>(static_cast<long long>(nSpheres) * (thread + 1) / nThreads);

#pragma omp single
        {
            buffers.resize(nThreads);
            for (auto &output: taskOutputs)
                output.Resize(nThreads);
            printf("Evolution (%d thread(s))\n", nThreads);
        }
        for (int i = sphereBegin; i < sphereEnd; i++)
            sphereOwner[i] = thread;
        buffers[thread].data = new Contact[contactCapacity];
        buffers[thread].forces.resize(nThreads);
        buffers[thread].xsis.resize(nThreads);
#pragma omp barrier

        do {
            // ---- Anticipation des positions (demi-pas) ----
#pragma omp single
            {
                solids->configuration.TIME += dt;
                Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows,
                                    solids->configuration.TIME, dt / 2, solids->spheres, solids->gravity);
                solids->configuration.mas->Move(dt / 2);
            }
            for (int i = sphereBegin; i < sphereEnd; i++) {
                spheres[i].move(dt / 2);
                // Remise a zero des forces et permutation des elongations : rien, jusqu'a la
                // detection, ne lit les forces ni les elongations des spheres
                spheres[i].initTimeStep();
            }
            {
                // Reinitialisation des contacts utilises au pas precedent par ce thread
                ContactBuffer &buffer = buffers[thread];
                for (int i = 0; i < buffer.previous; ++i)
                    buffer.data[i].TimeStepInitialization();
                buffer.count = 0;
                for (auto &list: buffer.forces)
                    list.clear();
                for (auto &list: buffer.xsis)
                    list.clear();
            }
#pragma omp barrier
#pragma omp single
            {
                Move::upDateHollowBall(solids->hollowBalls, dt);
                solids->gravity.Move(solids->configuration.TIME, dt / 2);
                Move::moveBodies(solids->bodies, dt / 2, solids->spheres);
                PeriodicityPL(solids->spheres, solids->plans);

                // Linked Cells
                LinkedCellFiller::Fill(solids->spheres, solids->configuration, cell);

                // Initialisation du pas de temps (hors spheres, faites plus haut en parallele)
                for (auto &body: solids->bodies)
                    body.TimeStepInitialization();
                for (auto &plan: solids->plans)
                    plan.TimeStepInitialization();
                for (auto &disk: solids->disks)
                    disk.TimeStepInitialization();
                for (auto &cone: solids->cones)
                    cone.TimeStepInitialization();
            }
            timer.Mark(0, thread);

            // ---- Detection des contacts (blocs ordonnes, repartition dynamique) ----
#pragma omp for schedule(dynamic, 1)
            for (int t = 0; t < nTasks; t++) {
                ContactBuffer &buffer = buffers[thread];
                const int start = buffer.count;
                RunDetectionTask(tasks[t], buffer.data, buffer.count, cell);
                taskOutputs[t].buffer = thread;
                taskOutputs[t].start = start;
                taskOutputs[t].count = buffer.count - start;
            }

            timer.Mark(1, thread);
            // ---- Calcul des forces (independant pour chaque contact) ----
            // puis traduction en contributions compactes, pendant que les contacts sont en cache
#pragma omp for schedule(dynamic, 1)
            for (int t = 0; t < nTasks; t++) {
                TaskOutput &out = taskOutputs[t];
                ContactBuffer &mine = buffers[thread];
                out.contributionBuffer = thread;
                for (int o = 0; o < nThreads; o++) {
                    out.forceStart[o] = static_cast<int>(mine.forces[o].size());
                    out.xsiStart[o] = static_cast<int>(mine.xsis[o].size());
                }
                if (out.count > 0) {
                    Contact *first = buffers[out.buffer].data + out.start;
                    ComputeForce::Compute(first, out.count, solids->configuration);
                    if (nThreads > 1)
                        ComputeForce::BuildContributions(first, out.count, sphereOwner, mine.forces, mine.xsis);
                }
                for (int o = 0; o < nThreads; o++) {
                    out.forceCount[o] = static_cast<int>(mine.forces[o].size()) - out.forceStart[o];
                    out.xsiCount[o] = static_cast<int>(mine.xsis[o].size()) - out.xsiStart[o];
                }
            }

            timer.Mark(2, thread);
            // ---- Sommation : chaque thread applique, bloc apres bloc dans l'ordre sequentiel,
            //      les contributions des objets dont il est proprietaire ----
            if (nThreads > 1) {
                for (int t = 0; t < nTasks; t++) {
                    const TaskOutput &out = taskOutputs[t];
                    const ContactBuffer &src = buffers[out.contributionBuffer];
                    ComputeForce::ApplyContributions(src.forces[thread].data() + out.forceStart[thread],
                                                     out.forceCount[thread],
                                                     src.xsis[thread].data() + out.xsiStart[thread],
                                                     out.xsiCount[thread]);
                }
            } else {
                // Un seul thread : sommation directe, bloc apres bloc (meme ordre)
                for (int t = 0; t < nTasks; t++) {
                    const TaskOutput &out = taskOutputs[t];
                    if (out.count > 0)
                        ComputeForce::SumForceAndMomentum(buffers[out.buffer].data + out.start, out.count);
                }
            }
            buffers[thread].previous = buffers[thread].count;
#pragma omp barrier
            timer.Mark(3, thread);
#pragma omp single
            {

                Move::UpDateForceContainer(solids->spheres, solids->plans, solids->disks, solids->cones,
                                           solids->configuration.TIME, dt, solids->gravity);
                solids->configuration.mas->getForces();
            }
            for (int i = sphereBegin; i < sphereEnd; i++)
                spheres[i].upDateVelocity(dt, solids->gravity, 0.0);
#pragma omp barrier
#pragma omp single
            {
                Move::upDateVelocityBodies(solids->bodies, solids->gravity, dt, solids->spheres);
                Move::upDateVelocityContainer(solids->plans, solids->disks, solids->cones, solids->elbows,
                                              solids->configuration.TIME, dt, solids->gravity);
                solids->configuration.mas->UpDateVelocity(dt);

                // Move
                Move::moveContainer(solids->plans, solids->disks, solids->cones, solids->elbows,
                                    solids->configuration.TIME, dt / 2, solids->spheres, solids->gravity);
                solids->configuration.mas->Move(dt / 2);
            }
            for (int i = sphereBegin; i < sphereEnd; i++)
                spheres[i].move(dt / 2);
#pragma omp barrier
#pragma omp single
            {
                Move::moveBodies(solids->bodies, dt / 2, solids->spheres);
                Move::upDateHollowBall(solids->hollowBalls, dt);

                solids->gravity.Move(solids->configuration.TIME, dt / 2);

                PeriodicityPL(solids->spheres, solids->plans);

                keepGoing = solids->configuration.TIME <= solids->configuration.Total - solids->configuration.dt * 0.99;

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

                        if (solids->configuration.outContact >= 1) {
                            // Contacts rassembles dans l'ordre sequentiel
                            contactsForOutput.clear();
                            for (int t = 0; t < nTasks; t++) {
                                const TaskOutput &out = taskOutputs[t];
                                contactsForOutput.insert(contactsForOutput.end(),
                                                         buffers[out.buffer].data + out.start,
                                                         buffers[out.buffer].data + out.start + out.count);
                            }
                            int nOut = static_cast<int>(contactsForOutput.size());
                            if (solids->configuration.outContact == 1 || solids->configuration.outContact > 2)
                                ReadWrite::writeOutContact(name, Ntp, nOut, contactsForOutput.data(),
                                                           solids->configuration);
                            if (solids->configuration.outContact >= 2)
                                ReadWrite::writeOutContactDetails(name, Ntp, nOut, contactsForOutput.data(),
                                                                  solids->configuration);
                        }
                        printf("Save File %d\t\ttime = %e\r", Ntp, solids->configuration.TIME);
                        fflush(stdout);
                        Ntp++;
                        if (isMonitoringActivated) {
                            Monitoring::getInstance().metrics(solids->configuration.TIME, solids->configuration.Total);
                        }
                    }
                }
            }
            timer.Mark(4, thread);
        } while (keepGoing);
#pragma omp barrier
        delete[] buffers[thread].data;
        buffers[thread].data = nullptr;
    }
    timer.Print();
    printf("\n");
    return Ntp;
}
