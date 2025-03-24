#pragma once

#include <vector>

class Plan;

class PlanR;

class Cone;

class Elbow;

class Sphere;

class Data;

class Body;

class HollowBall;

class Contact;

class Gravity;

class Compaction {
public:
    static void
    Secousse(std::vector<Plan> &pl, std::vector<PlanR> &plr, std::vector<Cone> &co, int Npl, int Nplr, int Nco,
             double Gamma, double f, Data &dat) noexcept;

    static void
    Relaxation(std::vector<Plan> &pl, std::vector<PlanR> &plr, std::vector<Cone> &co, int Npl, int Nplr, int Nco,
               Data &dat) noexcept;

    static int Run(int &Npl, int &Nplr, int &Nco, int &Nelb, int &Nsph, int &Nsph0, int &Nbd, int &Nhb, int &Nct,
                   std::vector<Plan> &pl, std::vector<PlanR> &plr, std::vector<Cone> &co, std::vector<Elbow> &elb,
                   std::vector<Sphere> &sph, std::vector<Body> &bd, std::vector<HollowBall> &hb, Contact *ct, Data &dat,
                   Gravity &gf,
                   Sphere *cell[], int &Ntp, char *name, bool record, int ntpi, int ntpf, double Gamma, double f,
                   int Nthreshold, bool isMonitoringActivated) noexcept;

    static int
    RunOMP(int &Npl, int &Nplr, int &Nco, int &Nelb, int &Nsph, int &Nsph0, int &Nbd, int &Nhb, int &Nct, int &Ncta,
           int &Nctb, int &Nctc,
           std::vector<Plan> &pl, std::vector<PlanR> &plr, std::vector<Cone> &co, std::vector<Elbow> &elb,
           std::vector<Sphere> &sph, std::vector<Body> &bd, std::vector<HollowBall> &hb, Contact *ct, Contact *cta,
           Contact *ctb, Contact *ctc, Data &dat, Gravity &gf,
           Sphere *cell[], int &Ntp, char *name, bool record, int ntpi, int ntpf, double Gamma, double f,
           int Nthreshold, bool isMonitoringActivated) noexcept;
};
