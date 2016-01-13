// cl -nologo main.cpp /link -out:main.exe
#define SIM_IMPLEMENTATION
#include "sim.h"
#include <stdio.h>

int main()
{
    sim_init(0, 0);

    sim_Command cmd;
    cmd.type = sim_CommandType_LandInFrontOf;
    cmd.x = 0;
    cmd.y = 0;
    cmd.i = 0;
    for (int i = 0; i < 1024; i++)
    {
        sim_State state;
        for (u32 tick = 0; tick < 100; tick++)
        {
            state = sim_tick(cmd);
            if (state.drone_cmd_done)
            {
                cmd.type = sim_CommandType_NoCommand;
            }
        }

        printf("time: %.2f seconds\n", state.elapsed_time);
        printf("         GREEN        \n");
        printf("+--------------------+\n");
        for (s32 yi = 0; yi < 20; yi++)
        {
            printf("|");
            for (s32 xi = 0; xi < 20; xi++)
            {
                s32 x_grid = xi;
                s32 y_grid = 19 - yi;
                bool target_at_xy = false;
                bool obstacle_at_xy = false;
                for (u32 target = 0; target < Num_Targets; target++)
                {
                    if (state.target_removed[target])
                        continue;
                    s32 xt = (s32)state.target_x[target];
                    s32 yt = (s32)state.target_y[target];
                    if (x_grid == xt && y_grid == yt)
                        target_at_xy = true;
                }
                for (u32 obstacle = 0; obstacle < Num_Obstacles; obstacle++)
                {
                    s32 xt = (s32)state.obstacle_x[obstacle];
                    s32 yt = (s32)state.obstacle_y[obstacle];
                    if (x_grid == xt && y_grid == yt)
                        obstacle_at_xy = true;
                }
                if (obstacle_at_xy)
                    printf("%c", 19);
                else if (target_at_xy)
                    printf("%c", 2);
                else
                    printf(" ");
            }
            printf("| ");
            if (yi < Num_Targets)
            {
                printf("target[%d]: %.2f\t%.2f\t%.2f",
                       yi,
                       state.target_x[yi],
                       state.target_y[yi],
                       state.target_q[yi]);
            }
            else if (yi - Num_Targets < Num_Obstacles)
            {
                int index = yi - Num_Targets;
                printf("obstacle[%d]: %.2f\t%.2f\t%.2f",
                       index,
                       state.obstacle_x[index],
                       state.obstacle_y[index],
                       state.obstacle_q[index]);
            }
            printf("\n");
        }
        printf("+--------------------+\n");
        printf("          RED         \n");
        printf("drone_cmd_done: %d\n", state.drone_cmd_done);
        printf("drone x: %.2f\n", state.drone_x);
        printf("drone y: %.2f\n", state.drone_y);
        getc(stdin);
        system("cls");
    }
    return 0;
}
