#define SIM_IMPLEMENTATION
#define SIM_CLIENT_CODE
#include "sim.h"
#include "gui.h"
#include <stdio.h>

enum ai_State
{
    ai_State_1,
    ai_State_2,
    ai_State_3
};

int main()
{
    sim_init_msgs(true);

    ai_State ai_state = ai_State_1;

    sim_State state;
    bool running = true;
    while (running)
    {
        sim_recv_state(&state);
        printf("Recv state %.2f\n", state.elapsed_time);
        sim_Observed_State observed = sim_observe_state(state);

        switch (ai_state)
        {
            case ai_State_1:
            {
                sim_Command cmd;
                cmd.type = sim_CommandType_LandInFrontOf;
                cmd.i = 5;
                sim_send_cmd(&cmd);
                ai_state = ai_State_2;

            } break;
            case ai_State_2:
            {
                if (state.drone.cmd_done)
                {
                    sim_Command cmd;
                    cmd.type = sim_CommandType_LandOnTopOf;
                    cmd.i = 2;
                    sim_send_cmd(&cmd);
                    ai_state = ai_State_3;
                }
            } break;
            case ai_State_3:
            {
                if (state.drone.cmd_done)
                {
                    sim_Command cmd;
                    cmd.type = sim_CommandType_Search;
                    cmd.x = 3.0f;
                    cmd.y = 3.0f;
                    sim_send_cmd(&cmd);
                }
            } break;
        }
    }

    return 0;
}
