#include <limits>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <vector>

struct Output 
{
    double brake_torque;
    double apps;
};

struct Parameters
{
    double Cx;
    double Cz;
    double Sx;
    double Sz;
    double RDRY;
    double X;
    double Z;
    double DELTA;
    double J0;
    //double lookahead;
    double RATIO_DIFF;
    double VEH_MASS;
    double R_WHEEL;
    double R_MEAN_BRAKE_DISK;
    double MU_BRAKE;
    //double M_PI;
    double D_PIST;
    double L_PED_UP;
    double L_PED_DOWN;
    double D_TILTON;
    double D_CARRUCOLA;
    double RIDUTTORE;
    double EFF_RIDUTTORE;
    double T_MAX_BRAKE;

    std::vector<float> gear_ratios = 
    {
        std::numeric_limits<int>::infinity(),
        (float)77/14,
        77/19,
        77*36/39/21,
        77*36/39/24
    };
    std::vector<unsigned short> NMOVET = 
    {
        10, 2500, 3500, 4500, 5500, 6500, 7500, 8500, 9500, 10500, 11500, 18000
    };
    std::vector<unsigned short> CDC = 
    {
        23, 23, 36, 40, 38, 44, 46, 52, 56, 51, 47, 47
    };
};

Output aps_brake_from_accl(double accl, int gear, int rpm, const Parameters& params)
{
    constexpr double G = 9.81;
    constexpr double air_density = 1.225;
    double Cx = params.Cx;
    double Cz = params.Cz;
    double Sx = params.Sx;
    double Sz = params.Sz;
    double RDRY = params.RDRY;
    double X = params.X;
    double Z = params.Z;
    double DELTA = params.DELTA;
    double J0 = params.J0;
    //double lookahead = params.lookahead;
    double RATIO_DIFF = params.RATIO_DIFF;

    double VEH_MASS  = params.VEH_MASS;
    double R_WHEEL = params.R_WHEEL;
    double R_MEAN_BRAKE_DISK = params.R_MEAN_BRAKE_DISK;
    double MU_BRAKE = params.MU_BRAKE;
    //double M_PI = params.M_PI;
    double D_PIST = params.D_PIST;
    double L_PED_UP = params.L_PED_UP;
    double L_PED_DOWN = params.L_PED_DOWN;
    double D_TILTON = params.D_TILTON;
    double D_CARRUCOLA = params.D_CARRUCOLA;
    double RIDUTTORE = params.RIDUTTORE;
    double EFF_RIDUTTORE = params.EFF_RIDUTTORE;
    double T_MAX_BRAKE = params.T_MAX_BRAKE;

    auto& gear_ratios = params.gear_ratios;
    auto& NMOVET = params.NMOVET;
    auto& CDC = params.CDC;

    double coppia_ruote = (X * RDRY) + (((VEH_MASS * G) + Z) * DELTA) + ((VEH_MASS + ((4.0 * J0) / pow(RDRY, 2)))) * RDRY * accl;
    float coppia_motore = 0.0f;
    float coppia_freno = 0.0f;
    float APS = 0.0f;

    if(coppia_ruote >= 0.0)
    {
        coppia_motore = (float)(coppia_ruote / RATIO_DIFF / gear_ratios[gear]);
        uint8_t RPMBOUNDINF = 11, RMPBOUNDSUP = 11;
        double CDCINTERPOLATO = 0.0, m = 0.0, q = 0.0;
        uint8_t i;

        for(i = 1; i < NMOVET.size(); ++i)
        {
            if(rpm <= NMOVET[i])
            {
                RPMBOUNDINF = i -1;
                RMPBOUNDSUP = i;
                break;
            }
        }

        m = (double)(
            (double)(CDC[RMPBOUNDSUP] - CDC[RPMBOUNDINF])
            / 
            (double)(NMOVET[RMPBOUNDSUP] - NMOVET[RPMBOUNDINF]));

        q = (double)(
            (double)(
                (double)(NMOVET[RMPBOUNDSUP] * CDC[RPMBOUNDINF])
                -
                (double)(NMOVET[RPMBOUNDINF] * CDC[RMPBOUNDSUP])
            )
            /
            (double)(NMOVET[RMPBOUNDSUP] - NMOVET[RPMBOUNDINF])
        );

        CDCINTERPOLATO = (double)(m * static_cast<double>(rpm)) + q;

        APS = (float)(coppia_motore / CDCINTERPOLATO);

        if(std::isnan(APS) || std::isinf(APS))
        {
            APS = 0.0f;
        }
    }
    else
    {
        APS = 0.0f;
        double T_mot_freno_perm = ((((((accl*VEH_MASS/3) * R_WHEEL/R_MEAN_BRAKE_DISK/4/MU_BRAKE/(M_PI/4 * pow(D_PIST,2)) * 10) * L_PED_DOWN/L_PED_UP*2*(M_PI/4*pow(D_TILTON,2))/10)*D_CARRUCOLA/2000)/RIDUTTORE*1000) / EFF_RIDUTTORE) / T_MAX_BRAKE * 1000;
        coppia_freno = (float)T_mot_freno_perm;

        coppia_freno = (coppia_freno < -100.0f) ? coppia_freno : 0.0f;
        coppia_freno = (coppia_freno < -300.0f) ? -300.0f : coppia_freno;

        coppia_freno /= 1000.0f;

        if(rpm <= 3200)
        {
            coppia_freno = 0.0f;
        }
    }

    Output output = {.brake_torque = coppia_freno, .apps = APS};
    return output;
}
