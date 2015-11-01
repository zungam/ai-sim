#ifndef _simulator_h_
#define _simulator_h_

// The simulator uses UDP packets for interprocess
// communication. Internally, the simulator polls
// for UDP packets containing a high-level command
// that the drone will execute. Furthermore, the
// simulator sends the complete world state at a
// predefined rate.

// To use these packets in your own application,
// simply include this file and use the functions
//  * sim_recv_state
//  * sim_send_cmd
// to receive state from the simulator or send a
// drone command to the simulator.

#define Num_Targets 10
#define Num_Obstacles 10
struct Robot
{
    float x; // x-coordinate in world coordinates
    float y; // y-coordinate in world coordinates
    float q; // rotation relative world x-axis
    float vl; // left wheel speed
    float vr; // right wheel speed
};

struct Drone
{
    float x;
    float y;
    float z;
};

struct SimulationState
{
    Robot targets[Num_Targets];
    Robot obstacles[Num_Obstacles];
};

// The drone command interface is still uncertain
// and will most likely change alot over time. So
// for now I provide an example of what it might
// look like.
enum DroneCmdType
{
    // land on the closest ground robot
    DroneCmdType_TouchTop = 0,

    // go to a given position (x, y)
    DroneCmdType_Goto
};

struct DroneCmd
{
    DroneCmdType type;
    float x;
    float y;
};

// Return: true if a new state is available
bool sim_recv_state(SimulationState *result);
void sim_send_cmd(DroneCmd *cmd);

#define UDP_IMPLEMENTATION
#include "udp.h"

#endif
