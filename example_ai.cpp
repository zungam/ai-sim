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
        sim_State state = {};
        if (sim_recv_state(&state))
        {
            printf("Got a state update!\n");
            printf("-------------------\n");

            printf("elapsed_sim_time = %.2f\n", state.elapsed_sim_time);
            printf("drone (x y z) = (%.2f %.2f %.2f)\n",
                   state.drone_x, state.drone_y, state.drone_z);

            for (int i = 0; i < Num_Targets; i++)
            {
                printf("targets[%d] (x y q vx vy) = (%.2f %.2f %.2f %.2f %.2f)\n", i, state.target_x[i], state.target_y[i], state.target_q[i], state.target_vx[i], state.target_vy[i]);
            }

            for (int i = 0; i < Num_Obstacles; i++)
            {
                printf("obstacles[%d] (x y q vx vy) = (%.2f %.2f %.2f %.2f %.2f)\n", i, state.obstacle_x[i], state.obstacle_y[i], state.obstacle_q[i], state.obstacle_vx[i], state.obstacle_vy[i]);
            }

            printf("\n\n");
        }

        // Let's send some data as well
        char input[256];
        printf("(1) Land on top of robot\n");
        printf("(2) Land in front of\n");
        printf("(3) Track robot\n");
        printf("(4) Search\n");
        printf("Enter a command [1/2/3/4]: ");
        scanf("%s", input);
        switch (input[0])
        {
            case '1':
            {
                int i;
                printf("i [0-9]: ");
                scanf("%d", &i);

                sim_Command cmd = {};
                cmd.type = sim_CommandType_LandOnTopOf;
                cmd.i = i;

                sim_send_cmd(&cmd);
            } break;
            case '2':
            {
                int i;
                printf("i [0-9]: ");
                scanf("%d", &i);

                sim_Command cmd = {};
                cmd.type = sim_CommandType_LandInFrontOf;
                cmd.i = i;

                sim_send_cmd(&cmd);
            } break;
            case '3':
            {
                int i;
                printf("i [0-9]: ");
                scanf("%d", &i);

                sim_Command cmd = {};
                cmd.type = sim_CommandType_Track;
                cmd.i = i;

                sim_send_cmd(&cmd);
            } break;
            case '4':
            {
                float x, y;
                printf("x y: ");
                scanf("%f %f", &x, &y);

                sim_Command cmd = {};
                cmd.type = sim_CommandType_Search;
                cmd.x = x;
                cmd.y = y;

                sim_send_cmd(&cmd);
            } break;
        }
    }

    udp_close();
}
