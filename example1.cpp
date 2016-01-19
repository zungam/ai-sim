#define SIM_IMPLEMENTATION
#define SIM_CLIENT_CODE
#include "sim.h"
#include "gui.h"
#include <stdio.h>

int main()
{
    sim_init_msgs(false);

    sim_State state;
    while (1)
    {
        sim_recv_state(&state);
        printf("Recv state %.2f\n", state.elapsed_time);
        sim_Observed_State observed = sim_observe_state(state);

        sim_Command cmd;
        cmd.type = sim_CommandType_LandInFrontOf;
        cmd.x = 0;
        cmd.y = 0;
        cmd.i = 0;

        sim_send_cmd(&cmd);
    }

    return 0;
}
