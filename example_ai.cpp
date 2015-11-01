#define SIM_CLIENT_CODE
#include "simulator.h"
#include <stdio.h>
int main(int argc, char **argv)
{
    // The boolean parameter specifies whether or not
    // we want the socket to be non-blocking. A blocking
    // socket will stall the calling thread until a packet
    // is available. Here I open a blocking socket, which
    // prevents the loop below from burning cycles.
    sim_init_msgs(false);

    // In your application you probably want to use
    // non-blocking sockets and poll once per roll
    // in your own main loop. Like so:

    // State latest_state
    // while (running)
    //     state_updated = false
    //     State state
    //     if (sim_recv(state))
    //         latest_data = state
    //         // Process new data if you need to
    //
    //     // ... Your AI algorithm here ...
    //

    // If your main loop is running much slower than the
    // send rate of packets, and you _need_ to process
    // all the packets that are sent, then you will need
    // to poll for and process packets on a seperate thread.

    while (1)
    {
        SimulationState state = {};
        if (sim_recv_state(&state))
        {
            printf("Got a state update!\n");
            printf("-------------------\n");

            printf("elapsed_sim_time = %.2f\n", state.elapsed_sim_time);
            printf("drone (x y z) = (%.2f %.2f %.2f)\n",
                   state.drone.x, state.drone.y, state.drone.z);

            for (int i = 0; i < Num_Targets; i++)
            {
                printf("targets[%d] (x y q vl vr) = (%.2f %.2f %.2f %.2f %.2f)\n", i, state.targets[i].x, state.targets[i].y, state.targets[i].q, state.targets[i].vl, state.targets[i].vr);
            }

            for (int i = 0; i < Num_Obstacles; i++)
            {
                printf("obstacles[%d] (x y q vl vr) = (%.2f %.2f %.2f %.2f %.2f)\n", i, state.obstacles[i].x, state.obstacles[i].y, state.obstacles[i].q, state.obstacles[i].vl, state.obstacles[i].vr);
            }

            printf("\n\n");
        }
    }

    udp_close();
}
