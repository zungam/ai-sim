#include "sim.h"

struct sim_State
    target x y angle
    obstacle x y angle
    drone x y
    elapsed_time
    used_seed

struct sim_Command
    x
    y
    i

    userdata

int main()
{
    while (1)
    {
        sim_State state = gui_recv(&state);

        sim_Command cmd = fancyAiAlgorithm(state);

        gui_send(cmd);
    }
}

sim_Command fancyAiAlgorithm(sim_State current_state)
{
    // simulate 10 times to see if something bad happens?
    bool bad_choice = false;
    for 0..10
    {
        sim_State state = current_state;

        // use a single land command
        sim_Command cmd;
        cmd.type = sim_CommandType_LandOnTopOf;
        cmd.i = 5;
        state = sim_tick(state, cmd);

        // otherwise follow
        cmd.type = sim_CommandType_Track;
        int steps = SecondsToTicks(5);
        for 0..steps
            state = sim_tick(state, cmd);

        // went over red line?
        if (state.target_y[5] < 0.0f)
            bad_choice = true;
    }

    if (!bad_choice)
    {
        return LandOnTopOf;
    }
    else
    {
        return NoCommand;
    }
}
