#include "../Includes/ComputingForce.h"

#include "../Includes/Configuration/Configuration.h"
#include "../Includes/Solids/Velocity.h"
#include "../Includes/Solids/Plan.h"
#include "../Includes/Solids/PlanR.h"
#include "../Includes/Solids/Cone.h"
#include "../Includes/Solids/Elbow.h"
#include "../Includes/Solids/Sphere.h"
#include "../Includes/Solids/Body.h"
#include "../Includes/Contact/Contact.h"
#include "../Includes/Elongations/Elongation.h"
#include "../Includes/Contact/ContactIdentifier.h"

#include <omp.h>

/* Fonction qui initialise tous les obejts avant chaque etape de calcul */
void ComputeForce::InitForTimeStep(const int &Nct, std::vector<Sphere> &sph, std::vector<Body> &bd, Contact *ct,
                                   std::vector<Plan> &pl, std::vector<PlanR> &plr, std::vector<Cone> &co,
                                   std::vector<Elbow> &elb) noexcept {
    for (auto &sphere: sph)
        sphere.initTimeStep();
    for (auto &body: bd)
        body.TimeStepInitialization();
    for (auto &plan: pl)
        plan.TimeStepInitialization();
    for (auto &disk: plr)
        disk.TimeStepInitialization();
    for (auto &cone: co)
        cone.TimeStepInitialization();
    for (int i = 0; i < Nct; ++i)
        ct[i].TimeStepInitialization();
}

/* Fonction qui initialise tous les obejts avant chaque etape de calcul en version parallel OMP */
void ComputeForce::InitForTimeStepOMP(const int &Nsph, const int &Nbd, const int &Nct, const int &Ncta, const int &Nctb,
                                      const int &Nctc, int &Npl, int &Nplr, int &Nco, int &Nelb,
                                      std::vector<Sphere> &sph, std::vector<Body> &bd, Contact *ct, Contact *cta,
                                      Contact *ctb, Contact *ctc, std::vector<Plan> &pl, std::vector<PlanR> &plr,
                                      std::vector<Cone> &co, std::vector<Elbow> &elb) noexcept {
    // Sphere
    for (int i = 0; i < Nsph; i++)
        sph[i].initTimeStep();
    // Bodies
    for (int i = 0; i < Nbd; i++)
        bd[i].TimeStepInitialization();
    // Ct
    for (int i = 0; i < Nct; i++)
        ct[i].TimeStepInitialization();
    // Cta si para 2
    for (int i = 0; i < Ncta; i++)
        cta[i].TimeStepInitialization();
    // Ctb si para 4
    for (int i = 0; i < Nctb; i++)
        ctb[i].TimeStepInitialization();
    // Ctc si para 4
    for (int i = 0; i < Nctc; i++)
        ctc[i].TimeStepInitialization();
    // Plan
    for (int i = 0; i < Npl; i++)
        pl[i].TimeStepInitialization();
    // Disque
    for (int i = 0; i < Nplr; i++)
        plr[i].TimeStepInitialization();
    // Cone
    for (int i = 0; i < Nco; i++)
        co[i].TimeStepInitialization();
}

void ComputeForce::SumForceAndMomentum(Contact *ct, const int Nct) noexcept {
    Contact *ctl = ct;
    for (int i = 0; i < Nct; i++) {
        switch (ctl->type) {
            default:
            case Contact::Type::None:
                break;
            case Contact::Type::SphereSphere:
            case Contact::Type::SphereHollowBall:
                ctl->sa->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                ctl->sb->GetElongationManager().AddXsi(ctl->xsi, ctl->id_b_xsi);
                ctl->sa->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->sa->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->sb->AddForce(ctl->Fbx, ctl->Fby, ctl->Fbz);
                ctl->sb->AddMomentum(ctl->Mbx, ctl->Mby, ctl->Mbz);
                break;
            case Contact::Type::SphereBody:
            case Contact::Type::BodyHollowBall:
                ctl->sa->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                ctl->bb->GetElongationManager().AddXsi(ctl->xsi, ctl->id_b_xsi);
                ctl->sa->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->sa->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->bb->AddForce(ctl->Fbx, ctl->Fby, ctl->Fbz);
                ctl->bb->AddMomentum(ctl->Mbx, ctl->Mby, ctl->Mbz);
                break;
            case Contact::Type::BodyBody:
                ctl->ba->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                ctl->bb->GetElongationManager().AddXsi(ctl->xsi, ctl->id_b_xsi);
                ctl->ba->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->ba->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->bb->AddForce(ctl->Fbx, ctl->Fby, ctl->Fbz);
                ctl->bb->AddMomentum(ctl->Mbx, ctl->Mby, ctl->Mbz);
                break;
            case Contact::Type::SpherePlan:
                ctl->sa->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->sa->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->pa->AddForce(ctl->Fbx, ctl->Fby, ctl->Fbz);
                ctl->pa->AddMomentum(ctl->Mbx, ctl->Mby, ctl->Mbz);
                ctl->sa->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                break;
            case Contact::Type::BodyPlan:
                ctl->ba->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->ba->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->pa->AddForce(ctl->Fbx, ctl->Fby, ctl->Fbz);
                ctl->pa->AddMomentum(ctl->Mbx, ctl->Mby, ctl->Mbz);
                ctl->ba->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                break;
            case Contact::Type::SpherePlanR:
                ctl->sa->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->sa->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->par->AddForce(ctl->Fbx, ctl->Fby, ctl->Fbz);
                ctl->par->AddMomentum(ctl->Mbx, ctl->Mby, ctl->Mbz);
                ctl->sa->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                break;
            case Contact::Type::BodyPlanR:
                ctl->ba->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->ba->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->par->AddForce(ctl->Fbx, ctl->Fby, ctl->Fbz);
                ctl->par->AddMomentum(ctl->Mbx, ctl->Mby, ctl->Mbz);
                break;
            case Contact::Type::SphereCone:
                ctl->sa->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->sa->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->cn->AddForce(ctl->Fbx, ctl->Fby, ctl->Fbz);
                ctl->cn->AddMomentum(ctl->Mbx, ctl->Mby, ctl->Mbz);
                ctl->sa->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                break;
            case Contact::Type::BodyCone:
                ctl->ba->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->ba->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->cn->AddForce(ctl->Fbx, ctl->Fby, ctl->Fbz);
                ctl->cn->AddMomentum(ctl->Mbx, ctl->Mby, ctl->Mbz);
                ctl->ba->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                break;
            case Contact::Type::SphereElbow:
                ctl->sa->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->sa->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->sa->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                break;
            case Contact::Type::BodyElbow:
                ctl->ba->AddForce(ctl->Fax, ctl->Fay, ctl->Faz);
                ctl->ba->AddMomentum(ctl->Max, ctl->May, ctl->Maz);
                ctl->ba->GetElongationManager().AddXsi(ctl->xsi, ctl->id_a_xsi);
                break;
        }
        ctl++;
    }
}


/* Fonction qui calcul les forces pour l'ensemble des contacts du tableau ct */
void ComputeForce::Compute(Contact *ct, const int Nct, Configuration &dat) noexcept {
    {
        double lax, lay, laz, lbx, lby, lbz;
        double Vax, Vay, Vaz, Vbx, Vby, Vbz;
        double wbx, wby, wbz;
        double Vn, Vtx, Vty, Vtz, Vt;
        double tx = 0, ty = 0, tz = 0;
        double meff, g0, N = 0, T = 0, gt;
        double Fx, Fy, Fz;
        //Data from dat
        double en = dat.en;
        double mu = dat.mu;
        double k = dat.k;
        double TIME = dat.TIME;
        double h = dat.dt;
        int ModelTg = dat.modelTg;
        Sphere *a, *b;
        Body *ba, *bb;
        int na, nb;
        Plan *p;
        PlanR *pr;
        Cone *cne;
        Elbow *elw;
        Contact *ctl = nullptr;
        for (int i = 0; i < Nct; i++) {
            ctl = &ct[i];
            N = 0;
            T = 0;
            switch (ctl->type) {
                case Contact::Type::None:
                    break;
                case Contact::Type::SphereSphere: //0
                    // Cas du contact entre deux spheres
                    a = ctl->sa;
                    b = ctl->sb;
                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = -a->Radius() * ctl->nx;
                    lay = -a->Radius() * ctl->ny;
                    laz = -a->Radius() * ctl->nz;
                    a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = b->Radius() * ctl->nx;
                    lby = b->Radius() * ctl->ny;
                    lbz = b->Radius() * ctl->nz;
                    b->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);

                    // Calcul des vitesses local du contact
                    computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    /* Cas du contact physique */
                    if (ctl->delta <= 0) {
                        //Calcul de la masse effective
                        meff = a->Mass() * b->Mass() / (a->Mass() + b->Mass());
                        // Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
                        g0 = DampingCoefficient(en, meff, k);
                        // Dynamique
                        if (ModelTg == 0) {
                            // Calcul du coefficient visqueux tangentiel
                            gt = 10000;
                            // Calcul de N et T
                            computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                        }
                        // Statique
                        else {
                            ctl->id_a_xsi = ContactIdentifier::computeIdentifier(
                                CONTACT_TYPE::SPHERE_VS_SPHERE, b->Num());
                            ctl->id_b_xsi = ContactIdentifier::computeIdentifier(
                                CONTACT_TYPE::SPHERE_VS_SPHERE, a->Num());
                            // Recherche de l'ancien xsi
                            ctl->xsi = a->GetElongationManager().FoundIt(ctl->id_a_xsi);
                            // Calcul de N, T et Xsi
                            computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                      dat.muS,
                                                      dat.muD);
                        }
                    }

                    // Calcul de la force de contact
                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                    ctl->Fax = Fx;
                    ctl->Fay = Fy;
                    ctl->Faz = Fz;
                    ctl->Max = lay * Fz - laz * Fy;
                    ctl->May = laz * Fx - lax * Fz;
                    ctl->Maz = lax * Fy - lay * Fx;

                    ctl->Fbx = -Fx;
                    ctl->Fby = -Fy;
                    ctl->Fbz = -Fz;
                    ctl->Mbx = lbz * Fy - lby * Fz; //-(lby*Fz-lbz*Fy);
                    ctl->Mby = lbx * Fz - lbz * Fx; //-(lbz*Fx-lbx*Fz);
                    ctl->Mbz = lby * Fx - lbx * Fy; //-(lbx*Fy-lby*Fx);
                    break;

                case Contact::Type::SphereBody: //10
                    // Cas du contact entre une sphere et une particules
                    a = ctl->sa;
                    bb = ctl->bb;
                    nb = ctl->nbb;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = -a->Radius() * ctl->nx;
                    lay = -a->Radius() * ctl->ny;
                    laz = -a->Radius() * ctl->nz;
                    a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);
                    std::tie(lbx, lby, lbz) = bb->Lever(ctl->nx, ctl->ny, ctl->nz, nb);
                    bb->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);

                    // Calcul des vitesses local du contact
                    computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    /* Cas du contact physique */
                    if (ctl->delta <= 0) {
                        //Calcul de la masse effective
                        meff = a->Mass() * bb->Mass() / (a->Mass() + bb->Mass());
                        // Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
                        g0 = DampingCoefficient(en, meff, k);
                        // Dynamique
                        if (ModelTg == 0) {
                            // Calcul du coefficient visqueux tangentiel
                            gt = 10000;
                            // Calcul de N et T
                            computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                        }
                        // Statique
                        else {
                            // Recherche de l'ancien xsi
                            ctl->id_a_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::SPHERE_VS_BODY, bb->Num(), nb);
                            ctl->id_b_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::SPHERE_VS_BODY, a->Num(), 10);
                            ctl->xsi = a->GetElongationManager().FoundIt(ctl->id_a_xsi);

                            // Calcul de N, T et Xsi
                            computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                      dat.muS,
                                                      dat.muD);
                        }
                    }

                    // Calcul de la force de contact
                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

                    ctl->Fax = Fx;
                    ctl->Fay = Fy;
                    ctl->Faz = Fz;
                    ctl->Max = lay * Fz - laz * Fy;
                    ctl->May = laz * Fx - lax * Fz;
                    ctl->Maz = lax * Fy - lay * Fx;

                    ctl->Fbx = -Fx;
                    ctl->Fby = -Fy;
                    ctl->Fbz = -Fz;
                    ctl->Mbx = lbz * Fy - lby * Fz; //-(lby*Fz-lbz*Fy);
                    ctl->Mby = lbx * Fz - lbz * Fx; //-(lbz*Fx-lbx*Fz);
                    ctl->Mbz = lby * Fx - lbx * Fy; //-(lbx*Fy-lby*Fx);
                    break;

                case Contact::Type::BodyBody: //20
                    // Cas du contact entre deux particules
                    ba = ctl->ba;
                    bb = ctl->bb;
                    na = ctl->nba;
                    nb = ctl->nbb;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    std::tie(lax, lay, laz) = ba->Lever(-ctl->nx, -ctl->ny, -ctl->nz, na);
                    ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    std::tie(lbx, lby, lbz) = bb->Lever(ctl->nx, ctl->ny, ctl->nz, nb);
                    bb->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);

                    // Calcul des vitesses local du contact
                    computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    /* Cas du contact physique */
                    if (ctl->delta <= 0) {
                        meff = ba->Mass() * bb->Mass() / (ba->Mass() + bb->Mass());
                        g0 = DampingCoefficient(en, meff, k);
                        // Dynamique
                        if (ModelTg == 0) {
                            gt = 10000;
                            // Calcul de N et T
                            computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                        }
                        // Statique
                        else {
                            ctl->id_a_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::BODY_VS_BODY, bb->Num(), na, nb);
                            ctl->id_b_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::BODY_VS_BODY, ba->Num(), nb, na);

                            ctl->xsi = ba->GetElongationManager().FoundIt(ctl->id_a_xsi);
                            computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                      dat.muS,
                                                      dat.muD);
                        }
                    }

                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

                    ctl->Fax = Fx;
                    ctl->Fay = Fy;
                    ctl->Faz = Fz;
                    ctl->Max = lay * Fz - laz * Fy;
                    ctl->May = laz * Fx - lax * Fz;
                    ctl->Maz = lax * Fy - lay * Fx;

                    ctl->Fbx = -Fx;
                    ctl->Fby = -Fy;
                    ctl->Fbz = -Fz;
                    ctl->Mbx = lbz * Fy - lby * Fz; //-(lby*Fz-lbz*Fy);
                    ctl->Mby = lbx * Fz - lbz * Fx; //-(lbz*Fx-lbx*Fz);
                    ctl->Mbz = lby * Fx - lbx * Fy; //-(lbx*Fy-lby*Fx);
                    break;
                case Contact::Type::SpherePlan: //1
                    // Cas du contact entre une sphere et un plan
                    a = ctl->sa;
                    p = ctl->pa;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = a->Radius() * ctl->nx;
                    lay = a->Radius() * ctl->ny;
                    laz = a->Radius() * ctl->nz;
                    a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = ctl->px - p->GetV().ox;
                    lby = ctl->py - p->GetV().oy;
                    lbz = ctl->pz - p->GetV().oz;

                    wbx = p->GetV().ValueOfWx(TIME);
                    wby = p->GetV().ValueOfWy(TIME);
                    wbz = p->GetV().ValueOfWz(TIME);
                    Vbx = p->GetV().ValueOfVx(TIME) + wby * lbz - wbz * lby;
                    Vby = p->GetV().ValueOfVy(TIME) + wbz * lbx - wbx * lbz;
                    Vbz = p->GetV().ValueOfVz(TIME) + wbx * lby - wby * lbx;

                    // Calcul des vitesses local du contact
                    computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);


                    /* Cas du contact physique */
                    if (ctl->delta <= 0) {
                        meff = a->Mass();
                        g0 = DampingCoefficient(en, meff, k);

                        // Dynamique
                        if (ModelTg == 0) {
                            gt = 10000;
                            // Calcul de N et T
                            computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                        }
                        // Statique
                        else {
                            ctl->id_a_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::SPHERE_VS_PLAN, p->Numero());
                            ctl->xsi = a->GetElongationManager().FoundIt(ctl->id_a_xsi);

                            computeContactForceStatic(ctl,  ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                      dat.muS,
                                                      dat.muD);
                        }
                    }
                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

                    ctl->Fax = -Fx;
                    ctl->Fay = -Fy;
                    ctl->Faz = -Fz;
                    ctl->Max = laz * Fy - lay * Fz;
                    ctl->May = lax * Fz - laz * Fx;
                    ctl->Maz = lay * Fx - lax * Fy;

                    ctl->Fbx = Fx;
                    ctl->Fby = Fy;
                    ctl->Fbz = Fz;
                    ctl->Mbx = lby * Fz - lbz * Fy;
                    ctl->Mby = lbz * Fx - lbx * Fz;
                    ctl->Mbz = lbx * Fy - lby * Fx;
                    break;
                case Contact::Type::BodyPlan: //11
                    // Cas du contact entre une particule et un plan
                    ba = ctl->ba;
                    na = ctl->nba;
                    p = ctl->pa;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = ba->SphereRadius(na) * ctl->nx + (ba->SphereX(na) - ba->X());
                    lay = ba->SphereRadius(na) * ctl->ny + (ba->SphereY(na) - ba->Y());
                    laz = ba->SphereRadius(na) * ctl->nz + (ba->SphereZ(na) - ba->Z());
                    ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = ctl->px - p->GetV().ox;
                    lby = ctl->py - p->GetV().oy;
                    lbz = ctl->pz - p->GetV().oz;
                    wbx = p->GetV().ValueOfWx(TIME);
                    wby = p->GetV().ValueOfWy(TIME);
                    wbz = p->GetV().ValueOfWz(TIME);
                    Vbx = p->GetV().ValueOfVx(TIME) + wby * lbz - wbz * lby;
                    Vby = p->GetV().ValueOfVy(TIME) + wbz * lbx - wbx * lbz;
                    Vbz = p->GetV().ValueOfVz(TIME) + wbx * lby - wby * lbx;

                    // Calcul des vitesses local du contact
                    computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    /* Cas du contact physique */
                    if (ctl->delta <= 0) {
                        meff = ba->Mass();
                        g0 = DampingCoefficient(en, meff, k);
                        // Dynamique
                        if (ModelTg == 0) {
                            gt = 10000;
                            // Calcul de N et T
                            computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                        }
                        // Statique
                        else {
                            ctl->id_a_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::BODY_VS_PLAN, p->Numero(), na);
                            ctl->xsi = ba->GetElongationManager().FoundIt(ctl->id_a_xsi);
                            computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                      dat.muS,
                                                      dat.muD);
                        }
                    }

                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

                    ctl->Fax = -Fx;
                    ctl->Fay = -Fy;
                    ctl->Faz = -Fz;
                    ctl->Max = laz * Fy - lay * Fz;
                    ctl->May = lax * Fz - laz * Fx;
                    ctl->Maz = lay * Fx - lax * Fy;

                    ctl->Fbx = Fx;
                    ctl->Fby = Fy;
                    ctl->Fbz = Fz;
                    ctl->Mbx = lby * Fz - lbz * Fy;
                    ctl->Mby = lbz * Fx - lbx * Fz;
                    ctl->Mbz = lbx * Fy - lby * Fx;
                    break;
                case Contact::Type::SpherePlanR: //2
                    // Cas du contact entre une sphere et un disque
                    a = ctl->sa;
                    pr = ctl->par;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = a->Radius() * ctl->nx;
                    lay = a->Radius() * ctl->ny;
                    laz = a->Radius() * ctl->nz;
                    a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = ctl->px - pr->GetV().ox;
                    lby = ctl->py - pr->GetV().oy;
                    lbz = ctl->pz - pr->GetV().oz;
                    wbx = pr->GetV().ValueOfWx(TIME);
                    wby = pr->GetV().ValueOfWy(TIME);
                    wbz = pr->GetV().ValueOfWz(TIME);
                    Vbx = pr->GetV().ValueOfVx(TIME) + wby * lbz - wbz * lby;
                    Vby = pr->GetV().ValueOfVy(TIME) + wbz * lbx - wbx * lbz;
                    Vbz = pr->GetV().ValueOfVz(TIME) + wbx * lby - wby * lbx;

                    // Calcul des vitesses local du contact
                    computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    /* Cas du contact physique */
                    if (ctl->delta < 0) {
                        meff = a->Mass();
                        g0 = DampingCoefficient(en, meff, k);

                        // Dynamique
                        if (ModelTg == 0) {
                            gt = 10000;
                            // Calcul de N et T
                            computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                        }
                        // Statique
                        else {
                            ctl->id_a_xsi = ContactIdentifier::computeIdentifier(
                                CONTACT_TYPE::SPHERE_VS_PLANR, pr->Numero());
                            ctl->xsi = a->GetElongationManager().FoundIt(ctl->id_a_xsi);

                            computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                      dat.muS,
                                                      dat.muD);
                        }
                    }
                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

                    ctl->Fax = -Fx;
                    ctl->Fay = -Fy;
                    ctl->Faz = -Fz;
                    ctl->Max = laz * Fy - lay * Fz;
                    ctl->May = lax * Fz - laz * Fx;
                    ctl->Maz = lay * Fx - lax * Fy;

                    ctl->Fbx = Fx;
                    ctl->Fby = Fy;
                    ctl->Fbz = Fz;
                    ctl->Mbx = lby * Fz - lbz * Fy;
                    ctl->Mby = lbz * Fx - lbx * Fz;
                    ctl->Mbz = lbx * Fy - lby * Fx;
                    break;

                case Contact::Type::BodyPlanR: //12
                    // Cas du contact entre une particule et un disque
                    ba = ctl->ba;
                    na = ctl->nba;
                    pr = ctl->par;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = ba->SphereRadius(na) * ctl->nx + (ba->SphereX(na) - ba->X());
                    lay = ba->SphereRadius(na) * ctl->ny + (ba->SphereY(na) - ba->Y());
                    laz = ba->SphereRadius(na) * ctl->nz + (ba->SphereZ(na) - ba->Z());
                    ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = ctl->px - pr->GetV().ox;
                    lby = ctl->py - pr->GetV().oy;
                    lbz = ctl->pz - pr->GetV().oz;
                    wbx = pr->GetV().ValueOfWx(TIME);
                    wby = pr->GetV().ValueOfWy(TIME);
                    wbz = pr->GetV().ValueOfWz(TIME);
                    Vbx = pr->GetV().ValueOfVx(TIME) + wby * lbz - wbz * lby;
                    Vby = pr->GetV().ValueOfVy(TIME) + wbz * lbx - wbx * lbz;
                    Vbz = pr->GetV().ValueOfVz(TIME) + wbx * lby - wby * lbx;

                    // Calcul des vitesses local du contact
                    computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    /* Cas du contact physique */
                    if (ctl->delta < 0) {
                        meff = ba->Mass();
                        g0 = DampingCoefficient(en, meff, k);

                        // Dynamique
                        if (ModelTg == 0) {
                            gt = 10000;
                            // Calcul de N et T
                            computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                        }
                        // Statique
                        else {
                            ctl->id_a_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::BODY_VS_PLANR, pr->Numero(), na);
                            ctl->xsi = ba->GetElongationManager().FoundIt(ctl->id_a_xsi);
                            computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                      dat.muS,
                                                      dat.muD);
                        }
                    }

                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                    ctl->Fax = -Fx;
                    ctl->Fay = -Fy;
                    ctl->Faz = -Fz;
                    ctl->Max = laz * Fy - lay * Fz;
                    ctl->May = lax * Fz - laz * Fx;
                    ctl->Maz = lay * Fx - lax * Fy;

                    ctl->Fbx = Fx;
                    ctl->Fby = Fy;
                    ctl->Fbz = Fz;
                    ctl->Mbx = lby * Fz - lbz * Fy;
                    ctl->Mby = lbz * Fx - lbx * Fz;
                    ctl->Mbz = lbx * Fy - lby * Fx;
                    break;

                case Contact::Type::SphereCone: //3
                    // Cas du contact entre une sphere et un cone
                    a = ctl->sa;
                    cne = ctl->cn;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = a->Radius() * ctl->nx;
                    lay = a->Radius() * ctl->ny;
                    laz = a->Radius() * ctl->nz;
                    a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = ctl->px - cne->GetV().ox;
                    lby = ctl->py - cne->GetV().oy;
                    lbz = ctl->pz - cne->GetV().oz;
                    wbx = cne->GetV().ValueOfWx(TIME);
                    wby = cne->GetV().ValueOfWy(TIME);
                    wbz = cne->GetV().ValueOfWz(TIME);
                    Vbx = cne->GetV().ValueOfVx(TIME) + wby * lbz - wbz * lby;
                    Vby = cne->GetV().ValueOfVy(TIME) + wbz * lbx - wbx * lbz;
                    Vbz = cne->GetV().ValueOfVz(TIME) + wbx * lby - wby * lbx;

                    // Calcul des vitesses local du contact
                    computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    meff = a->Mass();
                    g0 = DampingCoefficient(en, meff, k);

                    // Dynamique
                    if (ModelTg == 0) {
                        gt = 10000;
                        // Calcul de N et T
                        computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                    }
                    // Statique
                    else {
                        N = k * (-ctl->delta) - g0 * Vn;
                        if (N < 0) N = 0;
                        ctl->id_a_xsi = ContactIdentifier::computeIdentifier(
                            CONTACT_TYPE::SPHERE_VS_CONE, cne->Numero());
                        ctl->xsi = a->GetElongationManager().FoundIt(ctl->id_a_xsi);
                        computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                  dat.muS,
                                                  dat.muD);
                    }

                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                    ctl->Fax = -Fx;
                    ctl->Fay = -Fy;
                    ctl->Faz = -Fz;
                    ctl->Max = laz * Fy - lay * Fz;
                    ctl->May = lax * Fz - laz * Fx;
                    ctl->Maz = lay * Fx - lax * Fy;

                    ctl->Fbx = Fx;
                    ctl->Fby = Fy;
                    ctl->Fbz = Fz;
                    ctl->Mbx = lby * Fz - lbz * Fy;
                    ctl->Mby = lbz * Fx - lbx * Fz;
                    ctl->Mbz = lbx * Fy - lby * Fx;
                    break;
                case Contact::Type::BodyCone: //13
                    // Cas du contact entre une particule et un cone
                    ba = ctl->ba;
                    na = ctl->nba;
                    cne = ctl->cn;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = ba->SphereRadius(na) * ctl->nx + (ba->SphereX(na) - ba->X());
                    lay = ba->SphereRadius(na) * ctl->ny + (ba->SphereY(na) - ba->Y());
                    laz = ba->SphereRadius(na) * ctl->nz + (ba->SphereZ(na) - ba->Z());
                    ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = ctl->px - cne->GetV().ox;
                    lby = ctl->py - cne->GetV().oy;
                    lbz = ctl->pz - cne->GetV().oz;
                    wbx = cne->GetV().ValueOfWx(TIME);
                    wby = cne->GetV().ValueOfWy(TIME);
                    wbz = cne->GetV().ValueOfWz(TIME);
                    Vbx = cne->GetV().ValueOfVx(TIME) + wby * lbz - wbz * lby;
                    Vby = cne->GetV().ValueOfVy(TIME) + wbz * lbx - wbx * lbz;
                    Vbz = cne->GetV().ValueOfVz(TIME) + wbx * lby - wby * lbx;

                    // Calcul des vitesses local du contact
                    computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    meff = ba->Mass();
                    // Parametre du contact fonction de plan et de l'espece de grain en contact
                    g0 = DampingCoefficient(en, meff, k);

                    // Dynamique
                    if (ModelTg == 0) {
                        gt = 10000;
                        // Calcul de N et T
                        computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                    }
                    // Statique
                    else {
                        // Calcul de la composante normale de la force de contact
                        N = k * (-ctl->delta) - g0 * Vn;
                        if (N < 0) N = 0;
ctl->id_a_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::BODY_VS_CONE, cne->Numero(), na);
                        ctl->xsi = ba->GetElongationManager().FoundIt(ctl->id_a_xsi);
                        computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,
                                                  dat.muD);
                    }

                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                    ctl->Fax = -Fx;
                    ctl->Fay = -Fy;
                    ctl->Faz = -Fz;
                    ctl->Max = laz * Fy - lay * Fz;
                    ctl->May = lax * Fz - laz * Fx;
                    ctl->Maz = lay * Fx - lax * Fy;

                    ctl->Fbx = Fx;
                    ctl->Fby = Fy;
                    ctl->Fbz = Fz;
                    ctl->Mbx = lby * Fz - lbz * Fy;
                    ctl->Mby = lbz * Fx - lbx * Fz;
                    ctl->Mbz = lbx * Fy - lby * Fx;
                    break;
                case Contact::Type::SphereElbow: //4
                    // Cas du contact entre une sphere et un coude
                    a = ctl->sa;
                    elw = ctl->ew;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = -a->Radius() * ctl->nx;
                    lay = -a->Radius() * ctl->ny;
                    laz = -a->Radius() * ctl->nz;
                    a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = ctl->px - elw->V.ox;
                    lby = ctl->py - elw->V.oy;
                    lbz = ctl->pz - elw->V.oz;
                    wbx = elw->V.ValueOfWx(TIME);
                    wby = elw->V.ValueOfWy(TIME);
                    wbz = elw->V.ValueOfWz(TIME);
                    Vbx = elw->V.ValueOfVx(TIME) + wby * lbz - wbz * lby;
                    Vby = elw->V.ValueOfVy(TIME) + wbz * lbx - wbx * lbz;
                    Vbz = elw->V.ValueOfVz(TIME) + wbx * lby - wby * lbx;

                    // Calcul des vitesses local du contact
                    computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    meff = a->Mass();
                    g0 = DampingCoefficient(en, meff, k);

                    // Dynamique
                    if (ModelTg == 0) {
                        gt = 10000;
                        // Calcul de N et T
                        computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                    }
                    // Statique
                    else {
                        ctl->id_a_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::SPHERE_VS_ELBOW, elw->numero);
                        ctl->xsi = a->GetElongationManager().FoundIt(ctl->id_a_xsi);
                        computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,
                                                  dat.muD);
                    }

                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

                    ctl->Fax = -Fx;
                    ctl->Fay = -Fy;
                    ctl->Faz = -Fz;
                    ctl->Max = laz * Fy - lay * Fz;
                    ctl->May = lax * Fz - laz * Fx;
                    ctl->Maz = lay * Fx - lax * Fy;
                    break;
                case Contact::Type::BodyElbow: //14
                    // Cas du contact entre une particule et un coude
                    ba = ctl->ba;
                    na = ctl->nba;
                    elw = ctl->ew;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = -ba->SphereRadius(na) * ctl->nx + (ba->SphereX(na) - ba->X());
                    lay = -ba->SphereRadius(na) * ctl->ny + (ba->SphereY(na) - ba->Y());
                    laz = -ba->SphereRadius(na) * ctl->nz + (ba->SphereZ(na) - ba->Z());
                    ba->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = ctl->px - elw->V.ox;
                    lby = ctl->py - elw->V.oy;
                    lbz = ctl->pz - elw->V.oz;
                    wbx = elw->V.ValueOfWx(TIME);
                    wby = elw->V.ValueOfWy(TIME);
                    wbz = elw->V.ValueOfWz(TIME);
                    Vbx = elw->V.ValueOfVx(TIME) + wby * lbz - wbz * lby;
                    Vby = elw->V.ValueOfVy(TIME) + wbz * lbx - wbx * lbz;
                    Vbz = elw->V.ValueOfVz(TIME) + wbx * lby - wby * lbx;

                    // Calcul des vitesses local du contact
                    computeVelocity(Vbx, Vby, Vbz, Vax, Vay, Vaz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    meff = ba->Mass();
                    g0 = DampingCoefficient(en, meff, k);

                    // Dynamique
                    if (ModelTg == 0) {
                        gt = 10000;
                        // Calcul de N et T
                        computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                    }
                    // Statique
                    else {
                        ctl->id_a_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::BODY_VS_ELBOW, elw->numero, na);
                        ctl->xsi = ba->GetElongationManager().FoundIt(ctl->id_a_xsi);
                        computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0, dat.muS,
                                                  dat.muD);
                    }

                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);

                    ctl->Fax = -Fx;
                    ctl->Fay = -Fy;
                    ctl->Faz = -Fz;
                    ctl->Max = laz * Fy - lay * Fz;
                    ctl->May = lax * Fz - laz * Fx;
                    ctl->Maz = lay * Fx - lax * Fy;
                    break;
                case Contact::Type::SphereHollowBall:
                    // Cas du contact entre deux spheres
                    a = ctl->sa;
                    b = ctl->sb;
                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lax = -a->Radius() * ctl->nx;
                    lay = -a->Radius() * ctl->ny;
                    laz = -a->Radius() * ctl->nz;
                    a->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    lbx = -b->Radius() * ctl->nx;
                    lby = -b->Radius() * ctl->ny;
                    lbz = -b->Radius() * ctl->nz;

                    b->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);

                    // Calcul des vitesses local du contact
                    computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    /* Cas du contact physique */
                    if (ctl->delta <= 0) {
                        //Calcul de la masse effective
                        meff = a->Mass() * b->Mass() / (a->Mass() + b->Mass());
                        // Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
                        g0 = DampingCoefficient(en, meff, k);
                        // Dynamique
                        if (ModelTg == 0) {
                            // Calcul du coefficient visqueux tangentiel
                            gt = 10000;
                            // Calcul de N et T
                            computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                        }
                        // Statique
                        else {
                            // Recherche de l'ancien xsi
                             ctl->id_a_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::SPHERE_VS_HOLLOWBALL, b->Num());
                            ctl->id_b_xsi = ContactIdentifier::computeIdentifier(CONTACT_TYPE::SPHERE_VS_HOLLOWBALL, a->Num());
                            ctl->xsi = a->GetElongationManager().FoundIt(ctl->id_a_xsi);
                            // Calcul de N, T et Xsi
                            computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                      dat.muS,
                                                      dat.muD);
                        }
                    }
                    // Calcul de la force de contact
                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                    ctl->Fax = Fx;
                    ctl->Fay = Fy;
                    ctl->Faz = Fz;
                    ctl->Max = lay * Fz - laz * Fy;
                    ctl->May = laz * Fx - lax * Fz;
                    ctl->Maz = lax * Fy - lay * Fx;

                    ctl->Fbx = -Fx;
                    ctl->Fby = -Fy;
                    ctl->Fbz = -Fz;
                    ctl->Mbx = lbz * Fy - lby * Fz; //-(lby*Fz-lbz*Fy);
                    ctl->Mby = lbx * Fz - lbz * Fx; //-(lbz*Fx-lbx*Fz);
                    ctl->Mbz = lby * Fx - lbx * Fy; //-(lbx*Fy-lby*Fx);
                    break;
                case Contact::Type::BodyHollowBall: //6
                    // Cas du contact entre une sphere et une particules
                    a = ctl->sa;
                    bb = ctl->bb;
                    nb = ctl->nbb;

                    /* Calcul des leviers et des vitesses des points d'impacte de chaque corps */
                    lbx = -a->Radius() * ctl->nx;
                    lby = -a->Radius() * ctl->ny;
                    lbz = -a->Radius() * ctl->nz;
                    a->PointVelocity(Vbx, Vby, Vbz, lbx, lby, lbz);

                    lax = -bb->SphereRadius(nb) * ctl->nx + (bb->SphereX(nb) - bb->X());
                    lay = -bb->SphereRadius(nb) * ctl->ny + (bb->SphereY(nb) - bb->Y());
                    laz = -bb->SphereRadius(nb) * ctl->nz + (bb->SphereZ(nb) - bb->Z());
                    bb->PointVelocity(Vax, Vay, Vaz, lax, lay, laz);

                    // Calcul des vitesses local du contact
                    computeVelocity(Vax, Vay, Vaz, Vbx, Vby, Vbz, ctl, Vn, Vt, Vtx, Vty, Vtz);

                    /* Cas du contact physique */
                    if (ctl->delta <= 0) {
                        //Calcul de la masse effective
                        meff = a->Mass() * bb->Mass() / (a->Mass() + bb->Mass());
                        // Coefficient visqueux normale fonction de la masse effective meff, de la raideur k et de la restituion en
                        g0 = DampingCoefficient(en, meff, k);
                        // Dynamique
                        if (ModelTg == 0) {
                            // Calcul du coefficient visqueux tangentiel
                            gt = 10000;
                            // Calcul de N et T
                            computeContactForceDynamic(N, T, ctl, k, g0, gt, mu, Vn, Vt, Vtx, Vty, Vtz, tx, ty, tz);
                        }
                        // Statique
                        else {
                            ctl->id_a_xsi = ContactIdentifier::computeIdentifier(BODY_VS_HOLLOWBALL, bb->Num(), 10);
                            ctl->id_b_xsi = ContactIdentifier::computeIdentifier(BODY_VS_HOLLOWBALL, a->Num(), 10);
                            // Recherche de l'ancien xsi
                            ctl->xsi = bb->GetElongationManager().FoundIt(ctl->id_b_xsi);
                            // Calcul de N, T et Xsi
                            computeContactForceStatic(ctl, ctl->xsi, N, T, tx, ty, tz, Vn, Vt, Vtx, Vty, Vtz, h, k, g0,
                                                      dat.muS,
                                                      dat.muD);
                        }
                    }

                    // Calcul de la force de contact
                    ComputeContactForce(ctl, tx, ty, tz, T, N, Fx, Fy, Fz);
                    ctl->Fbx = Fx;
                    ctl->Fby = Fy;
                    ctl->Fbz = Fz;
                    ctl->Mbx = lay * Fz - laz * Fy;
                    ctl->Mby = laz * Fx - lax * Fz;
                    ctl->Mbz = lax * Fy - lay * Fx;

                    ctl->Fax = -Fx;
                    ctl->Fay = -Fy;
                    ctl->Faz = -Fz;
                    ctl->Max = lbz * Fy - lby * Fz; //-(lby*Fz-lbz*Fy);
                    ctl->May = lbx * Fz - lbz * Fx; //-(lbz*Fx-lbx*Fz);
                    ctl->Maz = lby * Fx - lbx * Fy; //-(lbx*Fy-lby*Fx);

                    break;
            }
        }
    }
}
