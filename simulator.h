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

    // Coordinate system is defined this way
    // x=+10      x=+10
    // y=+10      y=-10
    //   +----------+
    //   |  GREEN   | ^
    //   |          | |
    //   |          | | Positive x-axis
    //   |   RED    | |
    //   +----------+
    // x=-10      x=-10
    // y=+10      y=-10

    // Only targets in view get their fields updated.
    bool  target_in_view[Num_Targets];   // True if target currently in view
    bool  target_reversing[Num_Targets]; // True if target currently reversing
    float target_q[Num_Targets];         // Angle relative x-axis
    float obstacle_rel_x[Num_Obstacles]; // x coordinate relative drone
    float obstacle_rel_y[Num_Obstacles]; // y coordinate relative drone
    int   drone_tile_x;                  // x position in tiles
                                         // (i.e. integers from -10 to 10)
    int   drone_tile_y;                  // y position in tiles

    // This is set to true once the drone has successfully
    // finished its given command. Some commands, like track,
    // are never completed. LandOnTopOf and LandInFrontOf
    // are completed once the drone has landed, respectively,
    // on top of or in front of the desired robot.
    bool drone_cmd_complete;
};

// The drone command interface is still uncertain
// and will most likely change alot over time. So
// for now I provide an example of what it might
// look like.
enum sim_CommandType
{
    sim_CommandType_LandOnTopOf = 0, // trigger one 45 deg turn of robot (i)
    sim_CommandType_LandInFrontOf,   // trigger one 180 deg turn of robot (i)
    sim_CommandType_Track,           // follow robot (i) at a constant height
    sim_CommandType_Search           // ascend to 3 meters and go to (x, y)
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
