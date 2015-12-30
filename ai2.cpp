/* TODO: Probabilistic world state simulation?
struct p_State
{
    float e_target_x[Num_Targets];
    float ...?
    Probability target_x[Num_Targets]
    Probability target_y[Num_Targets]
}
p_expected_target_x(int i)
p_expected_target_q(int i)
...
p_sim_step(p_State, dt)

Interface with perception and control:
How to give command "tap target %d"?

Determine which action to take by simulating ahead:
    Performance measure includes how far it got to
    the green line, how far into the AVOID region it
    went, how many times it collided...
*/

#define SIM_CLIENT_CODE
#include "simulator.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#define for_each_target(it) for (int it = 0; it < Num_Targets; it++)
#define Two_Pi 6.28318530718f
#define One_Pi 3.14159265359f
#define Reverse_Interval 20.0f
#define Robot_Speed 0.33f

float frand()
{
    return (float)rand() / (float)RAND_MAX;
}

enum Action
{
    action_Rotate_45,
    action_Rotate_180,
    action_Follow,
    action_Return
};

enum ai_State
{
    ai_State_Start = 0,
    ai_State_Roaming,
    ai_State_Herding,
    ai_State_Tapping,
    ai_State_Ignoring,
};

struct GroundPos
{
    float x, y;
};

#define array_count(arr) (sizeof(arr) / sizeof(arr[0]))

struct BehaviourState
{
    float q;
    float x;
    float y;
};

float distance(BehaviourState a,
               BehaviourState b)
{
    float dq = min(abs(a.q - b.q), abs(a.q - Two_Pi - b.q));
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dp = dx*dx + dy*dy;

    float weight_q = 1.0f;
    float weight_p = 1.0f;
    float ds = weight_q*dq*dq + weight_p*dp*dp;
    return ds;
}

bool about_to_leave(float x, float y, float q,
                    float time_until_reverse,
                    float reversing)
{
    if (reversing)
        return false;
    float travel = time_until_reverse * Robot_Speed * sin(q);
    if (y + travel > 21.0f)
        return true;
    else
        return false;
}

Action select_action(float x, float y, float q,
                     float time_until_reverse, float reversing)
{
    #if 0
    struct Behaviour
    {
        // When state is close to:
        BehaviourState state;

        // Do:
        Action response;
    };

    Behaviour behaviours[] = {
        { { 0, 0, 0 }, action_Rotate_45 },
    };

    BehaviourState state = { x, y, q };
    int select_i = 0;
    float select_ds = distance(state, behaviours[0].state);
    for (int i = 1; i < array_count(behaviours); i++)
    {
        float ds = distance(state, behaviours[i].state);
        if (ds < select_ds)
        {
            select_ds = ds;
            select_i = i;
        }
    }

    return behaviours[select_i].response;
    #else

    if (reversing)
        return action_Follow;

    if (time_until_reverse < 5.0f)
        return action_Follow;

    if (about_to_leave(x, y, q, time_until_reverse, reversing))
        return action_Return;

    float q_deg = q * 180.0f / One_Pi;
    if (q_deg >= 260.0f || q_deg <= 60.0f)
    {
        return action_Rotate_180;
    }
    else if (q_deg >= 120.0f)
    {
        return action_Rotate_45;
    }
    else
    {
        return action_Follow;
    }
    #endif
}

float distance(GroundPos a, GroundPos b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrt(dx*dx + dy*dy);
}

#define Num_Waypoints 32
int closest_waypoint(GroundPos *waypoints, GroundPos p)
{
    int   result_i = 0;
    float result_d = distance(waypoints[0], p);
    for (int i = 1; i < Num_Waypoints; i++)
    {
        float dist = distance(waypoints[i], p);
        if (dist < result_d)
        {
            result_d = dist;
            result_i = i;
        }
    }
    return result_i;
}

int next_waypoint(int wp)
{
    return (wp + 1) % Num_Waypoints;
}

int main(int argc, char **argv)
{
    sim_init_msgs(false);

    // TODO: Uniformly distribute these to
    // most efficiently search the space,
    // including the corners of the grid.
    // For now, randomize max radius.
    GroundPos waypoints[Num_Waypoints];
    for (int i = 0; i < Num_Waypoints; i++)
    {
        float t = i / (float)(Num_Waypoints);
        float f = Two_Pi * t;
        float a = 2.0f * frand();
        waypoints[i].x = 10.0f + (6.6f + a) * cos(f);
        waypoints[i].y = 10.0f + (6.6f + a) * sin(f);
    }

    float last_elapsed_time = 0.0f;
    int current_waypoint = 0;
    int target_to_herd = 0;
    int running = 1;
    ai_State ai_state = ai_State_Start;

    // TODO: Observe this somehow
    float time_until_reverse = Reverse_Interval;

    while (running)
    {
        sim_State state = {};
        sim_recv_state(&state);

        float elapsed_time = state.elapsed_sim_time;
        float delta_time = elapsed_time - last_elapsed_time;
        last_elapsed_time = elapsed_time;

        time_until_reverse -= delta_time;
        if (time_until_reverse < 0.0f)
            time_until_reverse += Reverse_Interval;

        // TODO: Finalize this transformation
        // Do we want center of tile or tile corner?
        float drone_x, drone_y;
        tile_to_world(state.drone_tile_x,
                      state.drone_tile_y,
                      &drone_x, &drone_y);
        drone_x += 0.5f;
        drone_y += 0.5f;
        GroundPos drone_p = { drone_x, drone_y };

        // TODO: Select target which (most likely)
        // has the highest priority (facing away,
        // closer to edge, etc).

        bool found_target_to_herd = false;
        if (ai_state == ai_State_Roaming)
        {
            for_each_target(target)
            {
                if (state.target_in_view[target])
                {
                    printf("saw %d\n", target);
                    float x = drone_x + state.target_rel_x[target];
                    float y = drone_y + state.target_rel_y[target];
                    float q = state.target_q[target];
                    bool reversing = state.target_reversing[target];
                    if (about_to_leave(x, y, q, time_until_reverse, reversing))
                    {
                        continue;
                    }
                    else
                    {
                        found_target_to_herd = true;
                        target_to_herd = target;
                        break;
                    }
                }
            }
        }

        for_each_target(target)
        {
            if (state.target_in_view[target] &&
                state.target_reversing[target])
            {
                time_until_reverse = 20.0f;
            }
        }

        // TODO: Perception interface?
        bool lost_target = false;
        if (!state.target_in_view[target_to_herd])
        {
            lost_target = true;
        }

        bool near_waypoint = false;
        {
            #define Waypoint_Proximity 1.0f
            GroundPos wp = waypoints[current_waypoint];
            near_waypoint = (distance(wp, drone_p) <
                             Waypoint_Proximity);
        }

        bool cmd_complete = state.drone_cmd_complete;

        printf("time: %.2f seconds\n", elapsed_time);
        switch (ai_state)
        {
            case ai_State_Start:
            {
                current_waypoint = closest_waypoint(waypoints, drone_p);

                sim_Command cmd;
                cmd.type = sim_CommandType_Search;
                cmd.x = waypoints[current_waypoint].x;
                cmd.y = waypoints[current_waypoint].y;
                sim_send_cmd(&cmd);

                ai_state = ai_State_Ignoring;
            } break;

            case ai_State_Ignoring:
            {
                if (near_waypoint)
                {
                    current_waypoint = next_waypoint(current_waypoint);
                    sim_Command cmd;
                    cmd.type = sim_CommandType_Search;
                    cmd.x = waypoints[current_waypoint].x;
                    cmd.y = waypoints[current_waypoint].y;
                    sim_send_cmd(&cmd);

                    ai_state = ai_State_Roaming;
                    printf("goto %.2f %.2f\n", cmd.x, cmd.y);
                }
                else
                {
                    sim_Command cmd;
                    cmd.type = sim_CommandType_Search;
                    cmd.x = waypoints[current_waypoint].x;
                    cmd.y = waypoints[current_waypoint].y;
                    sim_send_cmd(&cmd);

                    ai_state = ai_State_Ignoring;
                    printf("goto %.2f %.2f\n", cmd.x, cmd.y);
                }
            } break;

            case ai_State_Roaming:
            {
                if (found_target_to_herd)
                {
                    sim_Command cmd;
                    cmd.type = sim_CommandType_Track;
                    cmd.i = target_to_herd;
                    sim_send_cmd(&cmd);

                    ai_state = ai_State_Herding;
                    printf("herd %d\n", target_to_herd);
                    break;
                }
                else if (near_waypoint)
                {
                    current_waypoint = next_waypoint(current_waypoint);
                    sim_Command cmd;
                    cmd.type = sim_CommandType_Search;
                    cmd.x = waypoints[current_waypoint].x;
                    cmd.y = waypoints[current_waypoint].y;
                    sim_send_cmd(&cmd);

                    ai_state = ai_State_Roaming;
                    printf("goto %.2f %.2f\n", cmd.x, cmd.y);
                }
                else
                {
                    sim_Command cmd;
                    cmd.type = sim_CommandType_Search;
                    cmd.x = waypoints[current_waypoint].x;
                    cmd.y = waypoints[current_waypoint].y;
                    sim_send_cmd(&cmd);

                    ai_state = ai_State_Roaming;
                    printf("goto %.2f %.2f\n", cmd.x, cmd.y);
                }
            } break;

            case ai_State_Herding:
            {
                if (lost_target)
                {
                    current_waypoint = closest_waypoint(waypoints,
                                                        drone_p);
                    sim_Command cmd;
                    cmd.type = sim_CommandType_Search;
                    cmd.x = waypoints[current_waypoint].x;
                    cmd.y = waypoints[current_waypoint].y;
                    sim_send_cmd(&cmd);

                    ai_state = ai_State_Roaming;
                    printf("goto %.2f %.2f\n", cmd.x, cmd.y);
                    break;
                }
                float x = drone_x + state.target_rel_x[target_to_herd];
                float y = drone_y + state.target_rel_y[target_to_herd];
                float q = state.target_q[target_to_herd];
                bool reversing = state.target_reversing[target_to_herd];
                Action action = select_action(x, y, q, time_until_reverse,
                                              reversing);
                switch (action)
                {
                    case action_Rotate_180:
                    {
                        sim_Command cmd;
                        cmd.type = sim_CommandType_LandInFrontOf;
                        cmd.i = target_to_herd;
                        sim_send_cmd(&cmd);

                        ai_state = ai_State_Tapping;
                        printf("r180 %.2f %.2f %.2f %d\n",
                               x, y, q, target_to_herd);
                    } break;

                    case action_Rotate_45:
                    {
                        sim_Command cmd;
                        cmd.type = sim_CommandType_LandOnTopOf;
                        cmd.i = target_to_herd;
                        sim_send_cmd(&cmd);

                        ai_state = ai_State_Tapping;
                        printf("r45 %.2f %.2f %.2f %d\n",
                               x, y, q, target_to_herd);
                    } break;

                    case action_Follow:
                    {
                        sim_Command cmd;
                        cmd.type = sim_CommandType_Track;
                        cmd.i = target_to_herd;
                        sim_send_cmd(&cmd);

                        ai_state = ai_State_Herding;
                        printf("follow %.2f %.2f %.2f %d\n",
                               x, y, q, target_to_herd);
                    } break;

                    case action_Return:
                    {
                        current_waypoint = closest_waypoint(waypoints,
                                                            drone_p);
                        sim_Command cmd;
                        cmd.type = sim_CommandType_Search;
                        cmd.x = waypoints[current_waypoint].x;
                        cmd.y = waypoints[current_waypoint].y;
                        sim_send_cmd(&cmd);

                        ai_state = ai_State_Ignoring;
                        printf("goto %.2f %.2f\n", cmd.x, cmd.y);
                    } break;
                }
            } break;

            case ai_State_Tapping:
            {
                // TODO: Lost target, timeout, cmd failed, ...
                if (lost_target)
                {
                    current_waypoint = closest_waypoint(waypoints,
                                                        drone_p);
                    sim_Command cmd;
                    cmd.type = sim_CommandType_Search;
                    cmd.x = waypoints[current_waypoint].x;
                    cmd.y = waypoints[current_waypoint].y;
                    sim_send_cmd(&cmd);

                    ai_state = ai_State_Roaming;
                    printf("goto %.2f %.2f\n", cmd.x, cmd.y);
                }
                else if (cmd_complete)
                {
                    sim_Command cmd;
                    cmd.type = sim_CommandType_Track;
                    cmd.i = target_to_herd;
                    sim_send_cmd(&cmd);

                    ai_state = ai_State_Herding;
                    printf("herd %d\n", cmd.i);
                }
                else
                {
                    printf("wait\n");
                }
            } break;
        }
    }

    udp_close();
}
