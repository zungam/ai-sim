#ifndef _simulator_h_
#define _simulator_h_
#define SIM_CLIENT_LISTEN_PORT 22334
#define SIM_SERVER_LISTEN_PORT 22335
#ifdef SIM_CLIENT_CODE
#define RECV_PORT SIM_CLIENT_LISTEN_PORT
#define SEND_PORT SIM_SERVER_LISTEN_PORT
#else
#define RECV_PORT SIM_SERVER_LISTEN_PORT
#define SEND_PORT SIM_CLIENT_LISTEN_PORT
#endif

// The simulator uses UDP packets for interprocess
// communication. Internally, the simulator polls
// for UDP packets containing a high-level command
// that the drone will execute. Furthermore, the
// simulator sends the complete world state at a
// predefined rate.

// To get state from the simulator and send drone
// commands to the simulator, do the following:
// #define SIM_CLIENT_CODE in your application,
// and include this file.

// The functions
//  - sim_init_msgs
//  - sim_recv_state
//  - sim_send_cmd
// can be used to communicate with the simulator.

// sim_recv_state will read as many packets as are
// available to acquire the latest one. This is to
// avoid the pipes clogging up if your loop is
// slower than the sending rate.

// For example usage, see example_ai.cpp.

#define Num_Targets 10
#define Num_Obstacles 4

// The simulation state is currently just fully
// observable. In the future we will want to be
// able to ask for state measurements that have
// been corrupted by noise and uncertainties.
struct sim_State
{
    float elapsed_sim_time;

    float target_x[Num_Targets];  // x position
    float target_y[Num_Targets];  // y position
    float target_vx[Num_Targets]; // x velocity
    float target_vy[Num_Targets]; // y velocity
    float target_q[Num_Targets];  // angle relative x-axis

    float obstacle_x[Num_Obstacles];  // x position
    float obstacle_y[Num_Obstacles];  // y position
    float obstacle_vx[Num_Obstacles]; // x velocity
    float obstacle_vy[Num_Obstacles]; // y velocity
    float obstacle_q[Num_Obstacles];  // angle relative x-axis

    float drone_x;     // x position
    float drone_vx;    // x velocity
    float drone_y;     // y position
    float drone_vy;    // y velocity
    float drone_z;     // height
    float drone_phi;   // roll
    float drone_theta; // pitch
    float drone_psi;   // yaw
};

// The drone command interface is still uncertain
// and will most likely change alot over time. So
// for now I provide an example of what it might
// look like.
enum sim_CommandType
{
    sim_CommandType_LandRobot = 0, // descend to target height
    sim_CommandType_LandFloor,     // descend to floor level
    sim_CommandType_Track,         // track robot number i
    sim_CommandType_Search         // ascend to 3 meters and go to (x, y)
};

struct sim_Command
{
    sim_CommandType type;
    float x;
    float y;
    int i;
};

#define UDP_IMPLEMENTATION
#include "udp.h"

void sim_init_msgs(bool nonblocking)
{
    udp_open(RECV_PORT, nonblocking);
}

bool sim_recv_state(sim_State *result)
{
    sim_State buffer = {};
    return udp_read_all((char*)result, (char*)&buffer, sizeof(sim_State), 0);
}

void sim_send_cmd(sim_Command *cmd)
{
    udp_addr dst = { 127, 0, 0, 1, SEND_PORT };
    udp_send((char*)cmd, sizeof(sim_Command), dst);
}

bool sim_recv_cmd(sim_Command *result)
{
    sim_Command buffer = {};
    return udp_read_all((char*)result, (char*)&buffer, sizeof(sim_Command), 0);
}

void sim_send_state(sim_State *state)
{
    udp_addr dst = { 127, 0, 0, 1, SEND_PORT };
    udp_send((char*)state, sizeof(sim_State), dst);
}

#endif
