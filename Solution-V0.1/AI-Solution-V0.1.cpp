#define SIM_IMPLEMENTATION
#define SIM_CLIENT_CODE
#include "../sim.h"
#include "../gui.h"
#include <stdio.h>
#include <iostream>

static double SPEED = 0.33;
static double GRID[22][22];

struct Plank
{
    float x_1;
    float y_1;
    float x_2;
    float y_2;
};


enum ai_State
{
    ai_landingOnTop,
    ai_landingInFront,
    ai_waiting,
    ai_tracking
};

void createGrid(){
    float grid[22][22];
    for(int x = 0; x < 22; x++) {
        for (int y = 0; y < 22; y++) {
            grid[x][y] = 0;
            if (x == 21 || x == 0 || y == 0) { grid[x][y] = -1000.0; }
            if (y == 21){
                grid[x][y] = 10000.0;
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


float gridValue(float x, float y)
{
    if (y>20) {
        return 10000;
    } else if (y < 0 || x < 0 || x > 20) {
        return 0;
    }
    y = abs(20-y);
    int x_0 = 10; //The peak's x-position
    int y_0 = 20; //The peak's y-position
    float y_v = 10; // The spread in y-direction
    float x_v = 7; // The spread in x-direction
    int amplitude = 100; // How "extreme" the values are
    float value = amplitude*exp(-((pow(x - x_0, 2) / (2 * pow(x_v, 2))) - ((pow(y - y_0, 2)) / (2 * pow(y_v, 2)))));
    return value;
}

void printValueGrid(){
    float grid[22][22];
    for(int x = 0; x < 22; x++) {
        for (int y = 0; y < 22; y++) {
            grid[x][y] = gridValue(x,y);
            std::cout << grid[x][y] << " ";
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
    plank.x_1 = check_ifInArena(timeToTurn*SPEED*cos(theta) + x);
    plank.y_1 = check_ifInArena(timeToTurn*SPEED*sin(theta) + y);
    plank.x_2 = check_ifInArena((timeToTurn - 20)*SPEED*cos(theta) + plank.x_1);
    plank.y_2 = check_ifInArena((timeToTurn - 20)*SPEED*sin(theta) + plank.y_1);
    return plank;
}

float findNumericIntegral(float (*f)(float x, float y), float a_x, float a_y, float b_x, float b_y, int n){
    float step_x = (a_x - b_x)/ n;  // width of each small rectangle
    float step_y = (a_y - b_y)/ n;  // width of each small rectangle
    float area = 0.0;  // signed area

    float dx = b_x - a_x;
    float dy = b_y - a_y;
    float dist = sqrt(dx*dx + dy*dy);

    for (int i = 0; i < n; i++) {
        area += f(a_x + (i + 0.5) * step_x, a_y + (i + 0.5) * step_y) * dist; // sum up each small rectangle
    }
    return area;
}

float findRobotValue(float x_robot, float y_robot, float theta, int timeToTurn)
{
    float reward1 = 0;
    float reward2 = 0;
    Plank positions = createPlank(x_robot, y_robot, theta, timeToTurn);
    // reward1 = gridValue(positions.x_1, positions.y_1);
    // reward2 = gridValue(positions.x_2, positions.y_2);
    reward1 = GRID[(int)positions.x_1][(int)positions.y_1];
    reward2 = GRID[(int)positions.x_2][(int)positions.y_2];

    //if(reward1 == gridValue(0,0) || reward1 == gridValue(20,20)){
    if(reward1 == GRID[0][21] || reward1 == GRID[0][0]){
        return 2*reward1;
    }
    //else if(reward2 == gridValue(0,0) || reward2 == gridValue(20,20)){
    if(reward1 == GRID[0][21] || reward1 == GRID[0][0]){
        return 2*reward2;
    }
    else{
        return reward1-reward2;
    }

}

bool target_inActionRange(sim_Observed_State observed_state, int target){
    float dx = observed_state.drone_x - observed_state.target_x[target];
    float dy = observed_state.drone_y - observed_state.target_y[target];
    if(sqrt(dx*dx + dy*dy) > Sim_Drone_Target_Proximity){
        return true;
    }
    return false;
}

bool targetIsMoving(int target, sim_Observed_State previous_state, sim_Observed_State observed_state){
    bool moving = true;
    if (previous_state.target_x[target] == observed_state.target_x[target] && 
        previous_state.target_y[target] == observed_state.target_y[target]) 
    {
        moving = false;
    }
    return moving;
}

//Pseudo ish kode
int choose_target(sim_Observed_State observed_state, sim_Observed_State previous_state){
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
        if(!observed_state.target_removed[i]){
            if (!targetIsMoving(i, previous_state, observed_state))
            {
                std::cout << "Target not moving" << std::endl;
                break;
            }
            Plank positions = createPlank(observed_state.target_x[i], observed_state.target_y[i],
            wrap_angle(observed_state.target_q[i] + 0.785), (int)observed_state.elapsed_time % 20);
            temp_value = findNumericIntegral(gridValue, positions.x_1, positions.y_1, positions.x_2, positions.y_2, 5);
            // temp_value = GRID[(int)observed_state.target_x[i]][(int)observed_state.target_y[i]];

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
            wrap_angle(state.target_q[i] + 0.785), (int)state.elapsed_time % 20); //0.785 radians is almost 45 degerees

    int rewardInFront = findRobotValue(state.target_x[i], state.target_y[i],
            wrap_angle(state.target_q[i] + 3.14), (int)state.elapsed_time % 20);

    int rewardForWait = findRobotValue(state.target_x[i], state.target_y[i],
            state.target_q[i], (int)state.elapsed_time % 20);
    std::cout << "Reward in front " << rewardInFront << std::endl;
    std::cout << "Reward on top " << rewardOnTop << std::endl;
    std::cout << "Reward wait" << rewardOnTop << std::endl;

    int max_reward = std::max(std::max(rewardInFront,rewardOnTop), rewardForWait);
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
    printValueGrid();
    sim_init_msgs(true);

    ai_State ai_state = ai_waiting;

    sim_State state;
    bool running = true;

    sim_Command cmd;
    sim_Observed_State observed_state;
    sim_Observed_State previous_state;
    int target = -1;
    float last_action_time;
    while (running)
    {
        sim_recv_state(&state);
        previous_state = observed_state;
        sim_Observed_State observed_state = sim_observe_state(state);

        if (observed_state.drone_cmd_done || ai_state == ai_tracking) //&& observed_state.elapsed_time - last_action_time >= 4.0)
        {
            last_action_time = observed_state.elapsed_time;

            if(target == -1){
                target = choose_target(observed_state, previous_state);
                ai_state = ai_tracking;
                cmd.type = sim_CommandType_Track;
                cmd.i = target;
                std::cout << "Tracking" << std::endl;
            }

            else if(target_inActionRange(observed_state, target) 
                    && targetIsMoving(target, previous_state, observed_state))
            {
                ai_state = choose_action(observed_state, target);
                switch (ai_state)
                {
                    case ai_landingOnTop:
                        cmd.type = sim_CommandType_LandOnTopOf;
                        cmd.i = target;
                        std::cout << "Top" << std::endl;
                        sim_send_cmd(&cmd);
                    break;
                    case ai_landingInFront:
                        cmd.type = sim_CommandType_LandInFrontOf;
                        cmd.i = target;
                        std::cout << "Front" << std::endl;
                        sim_send_cmd(&cmd);
                    break;
                    case ai_waiting:
                        std::cout << "WAITING" << std::endl;
                    break;
                target = -1;
                }
            }
        }
    }

    return 0;
}
