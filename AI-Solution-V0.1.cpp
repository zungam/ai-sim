#define SIM_IMPLEMENTATION
#define SIM_CLIENT_CODE
#include "sim.h"
#include "gui.h"
#include <stdio.h>
#include <iostream>

static double SPEED = 0.33;
static double GRID[22][22];

struct Plank
{
    float x_f;
    float y_f;
    float x_b;
    float y_b;
};


enum ai_State
{
    ai_landingOnTop,
    ai_landingInFront,
    ai_waiting
};


void createGrid(){
    float grid[22][22];
    for(int x = 0; x < 22; x++) {
        for (int y = 0; y < 22; y++) {
            grid[x][y] = 0;
            if (x == 21 || x == 0 || y == 0) { grid[x][y] = -1000.0; }
            if (y == 21){
                grid[x][y] = 2000.0;
            }
            
        }
    }
    //Setter indre verdier
    for (int n = 0; n < 5; n++) {
        //Starter i (1,1)
        for (int x = 1; x < 21; x++){
            for (int y = 1; y < 21; y++){
                float snitt = (grid[x + 1][y] + grid[x - 1][y] + grid[x][y + 1] + grid[x][y - 1]) / 4;
                grid[x][y] = snitt;
            }
        }
        //Starter i (1,20)
        for (int x = 1; x < 21; x++){
            for (int y = 20; y > 0; y--){
                float snitt = (grid[x + 1][y] + grid[x - 1][y] + grid[x][y + 1] + grid[x][y - 1]) / 4;
                grid[x][y] = snitt;
            }
        }
        // Starter i (20,1)
        for (int x = 20; x > 0; x--){
            for (int y = 1; y < 21; y++){
                float snitt = (grid[x + 1][y] + grid[x - 1][y] + grid[x][y + 1] + grid[x][y - 1]) / 4;
                grid[x][y] = snitt;
            }
        }
        //Starter i (20,20)
        for (int x = 20; x > 0; x--){
            for (int y = 20; y > 0; y--){
                float snitt = (grid[x + 1][y] + grid[x - 1][y] + grid[x][y + 1] + grid[x][y - 1]) / 4;
                grid[x][y] = snitt;
            }
        }
    }
    for(int i=0; i < 22; i++){

        for(int j=0; j < 22; j++){
            std::cout << grid[i][j] << " ";
            GRID[i][j] = grid[i][j]+1000;
        }
        std::cout << std::endl;
    }
}

int check_ifInArena(float x){
    if(x > 21){
        return 21;
    }
    else if(x<0){
        return 0;
    }
    else{
        return x;
    }
}

Plank createPlank(float x, float y, float theta, int timeToTurn)
{

    Plank plank;
    plank.x_f = check_ifInArena(timeToTurn*SPEED*cos(theta) + x);
    plank.y_f = check_ifInArena(timeToTurn*SPEED*sin(theta) + y);
    plank.x_b = check_ifInArena((timeToTurn - 20)*SPEED*cos(theta) + plank.x_f);
    plank.y_b = check_ifInArena((timeToTurn - 20)*SPEED*sin(theta) + plank.y_f);
    return plank;
}

float findRobotValue(float x_robot, float y_robot, float theta, int timeToTurn)
{
    float reward1 = 0;
    float reward2 = 0;
    Plank positions = createPlank(x_robot, y_robot, theta, timeToTurn);
    reward1 = GRID[(int)positions.x_f][(int)positions.y_f];
    reward2 = GRID[(int)positions.x_b][(int)positions.y_b];

    if(reward1 == 2000 || reward1 == -1000 ){
        return 2*reward1;
    }
    else if(reward2 == 2000 || reward2 == -1000){
        return 2*reward2;
    }
    else{
        return reward1+reward2;
    }

}

//Pseudo ish kode
int choose_target(sim_Observed_State state){
    int max_value = 0;
    int temp_value = 0;
    int target = 0;
    // for(int i = 0; i < Num_Targets; i++){
    //     if(!state.target_removed[i]){
    //         temp_value = findRobotValue(state.target_x[i], state.target_y[i],
    //             state.target_q[i], (int)state.elapsed_time % 20);

    //         if(temp_value < max_value){
    //             max_value = temp_value;
    //             target = i;
    //         }
    //     }

    // }
    for(int i = 0; i < Num_Targets; i++){
        if(!state.target_removed[i]){
            temp_value = GRID[(int)state.target_x[i]][(int)state.target_y[i]];

            if(temp_value > max_value){
                max_value = temp_value;
                target = i;
            }
        }
    }
    return target;
}

ai_State choose_action(sim_Observed_State state, int i){
    int rewardOnTop = findRobotValue(state.target_x[i], state.target_y[i],
            state.target_q[i] + 0.785, (int)state.elapsed_time % 20); //0.785 radians is almost 45 degerees

    int rewardInFront = findRobotValue(state.target_x[i], state.target_y[i],
            state.target_q[i] + 3.14, (int)state.elapsed_time % 20);

    int rewardForWait = findRobotValue(state.target_x[i], state.target_y[i],
            state.target_q[i], (int)state.elapsed_time % 20);

    int max_reward = std::max(std::max(rewardOnTop, rewardInFront), rewardForWait);
    if(max_reward == rewardForWait){
        return ai_waiting;
    }
    else if(max_reward == rewardInFront){
        return ai_landingInFront;
    }
    else if(max_reward == rewardOnTop){
        return ai_landingOnTop;
    }
    else{
        return ai_waiting;
    }
}

int main()
{
    createGrid(); 
    sim_init_msgs(true);

    ai_State ai_state = ai_waiting;

    sim_State state;
    bool running = true;
    sim_Command cmd;
    int target = 0;
    while (running)
    {
        sim_recv_state(&state);
        printf("Recv state %.2f\n", state.elapsed_time);
        sim_Observed_State observed = sim_observe_state(state);

        if (state.drone.cmd_done)
        {
            target = choose_target(observed);

            ai_state = choose_action(observed, target);

            switch (ai_state)
            {
                case ai_landingOnTop:
                    cmd.type = sim_CommandType_LandInFrontOf;
                    cmd.i = target;
                    sim_send_cmd(&cmd);
                break;
                case ai_landingInFront:
                    cmd.type = sim_CommandType_LandOnTopOf;
                    cmd.i = target;
                    sim_send_cmd(&cmd);
                break;
                case ai_waiting:
                    for(int i=0; i < 100000; i++){}
                    std::cout << "WAITING" << std::endl;
                break;
            }
        }
    }

    return 0;
}
