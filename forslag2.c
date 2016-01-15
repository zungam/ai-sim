#include "sim.h"

int main()
{
    sim_State RealSimState = sim_init()
    while (1)
    {
        sim_Command = fancyAiAlgorithm(RealSimState);
        RealSimState = sim_tick(RealSimState);
    }
}
