#define SIM_IMPLEMENTATION
#include "sim.h"
#include <stdio.h>

//This program is used to access the simulation from python. Needs to be compiled as a .dll for windows or .so for linux to work.
//The three functions used in python are initialize(), step() and send_command()
//step returns the observation, reward and 1 if done.


int step_length = 20; //Frames?
int last_robot_reward = 0;
int last_time = 0;
//AI actions
enum ai_Actions{
	ai_Search = 0,
	ai_LandOnTop,
	ai_Track,
	ai_Wait
}

//Input data for neural network
struct ai_data_input_struct{
	float elapsed_time;
	float drone_x;
	float drone_y;
	float target_x[Num_Targets];
	float target_y[Num_Targets];
}

struct step_result{
	ai_data_input_struct observation;
	float reward;
	int done;     
}

int get_sim_Num_Targets(){
	return Num_Targets;
}

ai_data_input_struct ai_data_input;
sim_State state;
sim_Observed_State observed_state;
sim_Command command;

//Interfacing(is that the right word here?) with python only works with C
//Atleast the way I do it

extern "C"{
	int initialize(){
		sim_init(12345);
		observed_state = sim_observe_state(state);
		//for reward calculation
		robots_inGoal = 0;
		robots_outOfBounds = 0;
	}

	float reward_calculator(){
		float result = 0;
    	int reward = 0;

    	//Robot rewards. Reward is added each time, so need to remove previously rewarded robots
    	for(int i = 0; i < Num_Targets; i++){
    	    reward += observed.target_reward[i];
    	}
    	result = reward_for_robot*(reward);

    	result -= last_robot_reward; 
    	last_robot_reward = result;

    	//Minus for amount of time spent
    	result -= (observed.elapsed_time - last_time);
    	last_time = observed.elapsed_time;

    	return result;
	}

	int get_done(){
		    int result = 0;
    	for(int i = 0; i < Num_Targets; i++){
    	    result += observed.target_removed[i];
    	}
    	if(result == Num_Targets){
    	    return 1;
    	}
    	return 0;
	}

	//Todo: returns robot No 1 right now
	int closest_robot(){
		return 0;
	}

	ai_data_input_struct update_ai_input(){
		ai_Observations result;
		result.elapsed_time = observed.elapsed_time;
		result.drone_x = observed.drone_x;
		result.drone_y = observed.drone_y;
		for(int i = 0; i < Num_Targets; i++){
		    if(observed.target_in_view[i]){
		        result.target_x[i] = observed.target_x[i];
		        result.target_y[i] = observed.target_y[i]; 
		    }
		}
		return result;
	}

	step_result step(){
	    step_result result;
	    for (unsigned int tick = 0; tick < step_length; tick++){
	        state = sim_tick(state, cmd);
	        observed = sim_observe_state(state);
	        if (observed.drone_cmd_done)
	        {
	            cmd.type = sim_CommandType_NoCommand;
	        }
	    }
	    result.observation = update_ai_input();
	    result.reward = reward_calculator();
	    result.done = get_done();
	}

	int send_command(int a){
		switch(a){
			case 0:
	            cmd.type = sim_CommandType_Search;
	            cmd.x = observed.target_x[closest_robot()];
	            cmd.y = observed.target_y[closest_robot()];
	        break;

	        case 1:
	            cmd.type = sim_CommandType_LandInFrontOf;
	            cmd.i = closest_robot();
	        break;
		}
	}
}