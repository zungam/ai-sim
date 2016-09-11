// Windows
// > cd build
// > vcvarsall
// > cl ../example2.cpp /link -out:example2.exe
// > example2

// Linux/osx
// $ cd build
// $ g++ ../example2.cpp -o example2
// $ ./example2

#define SIM_IMPLEMENTATION
#include "sim.h"
#include <stdio.h>

int main()
{
    sim_Observed_State state = sim_load_snapshot("snapshot729426942-148");

    printf("elapsed_time: %.2f seconds\n", state.elapsed_time);
    printf("drone_x: %.2f meters\n", state.drone_x);
    printf("drone_y: %.2f meters\n", state.drone_y);

    for (int i = 0; i < Num_Targets; i++)
    {
        printf("target %d\n", i);
        printf("\tin_view: %s\n", state.target_in_view[i]?"true":"false");
        printf("\treversing: %s\n", state.target_reversing[i]?"true":"false");
        printf("\tremoved: %s\n", state.target_removed[i]?"true":"false");
        printf("\tx: %.2f meters\n", state.target_x[i]);
        printf("\ty: %.2f meters\n", state.target_y[i]);
        printf("\tq: %.2f radians\n", state.target_q[i]);
    }

    for (int i = 0; i < Num_Obstacles; i++)
    {
        printf("obstacle %d\n", i);
        printf("\tx: %.2f meters\n", state.obstacle_x[i]);
        printf("\ty: %.2f meters\n", state.obstacle_y[i]);
        printf("\tq: %.2f radians\n", state.obstacle_q[i]);
    }

    return 0;
}
