#define SIM_CLIENT_CODE
#include "simulator.h"
#include <stdio.h>
int main(int argc, char **argv)
{
    // The boolean parameter specifies whether or not
    // we want the socket to be non-blocking. A blocking
    // socket will stall the calling thread until a packet
    // is available. Here I open a non-blocking socket, which
    // allows me to run on a loop without stalling for data.
    sim_init_msgs(true);

    while (1)
    {
        // Attempt to get the latest state data from simulator
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

        // Let's send some data as well
        char input[256];
        printf("Enter a command (g)oto/(s)earch: ");
        scanf("%s", input);
        if (input[0] == 'g')
        {
            float x, y;
            printf("x y: ");
            scanf("%f %f", &x, &y);

            DroneCmd cmd = {};
            cmd.type = DroneCmdType_Goto;
            cmd.x = x;
            cmd.y = y;

            sim_send_cmd(&cmd);
        }
        else if (input[0] == 's')
        {
            DroneCmd cmd = {};
            cmd.type = DroneCmdType_Search;

            sim_send_cmd(&cmd);
        }
    }

    udp_close();
}
