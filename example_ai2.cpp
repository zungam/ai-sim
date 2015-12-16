#define SIM_CLIENT_CODE
#include "simulator.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#define Tile_Dim 20
#define for_each_tile(it) for (int it = 0; it < Tile_Dim*Tile_Dim; it++)
#define for_each_target(it) for (int it = 0; it < Num_Targets; it++)

struct Belief
{
    float strength[Tile_Dim*Tile_Dim];
};

struct Target
{
    bool visible;
    bool identified;

    // The index into the original array of currently
    // visible robots that we received from the vision
    // processor. I should be able to tell the control
    // module to land on the robot of this index, and
    // it - together with the vision module - know which
    // robot in our view that I'm talking about.
    int vision_index;

    float last_seen_time;
    float last_seen_x;
    float last_seen_y;
    float last_seen_q;
    float time_until_reverse; // Estimated

    Belief belief;
};

struct Ai
{
    float time;
    float drone_x;
    float drone_y;
    Target targets[Num_Targets];
};

float belief_at(Belief *b, int x, int y)
{
    assert(x >= 0 && x < Tile_Dim && y >= 0 && y < Tile_Dim);
    return b->strength[y*Tile_Dim+x];
}

void belief_set(Belief *b, int x, int y, float p)
{
    if (x < 0 || x >= Tile_Dim || y < 0 || y >= Tile_Dim)
        return;
    b->strength[y*Tile_Dim+x] = p;
}

void belief_clear(Belief *b, float p)
{
    for_each_tile(i)
        b->strength[i] = p;
}

void belief_union(Belief *belief_a,
                  Belief *belief_b,
                  Belief *belief_result)
{
    // For any given cell, the probability that
    // a cell is believed to be occupied by a OR b
    // is given by
    // P(A v B) = P(A) + P(B) - P(A^B)
    for_each_tile(i)
    {
        float a = belief_a->strength[i];
        float b = belief_b->strength[i];
        belief_result->strength[i] = a+b-a*b;
    }
}

float total_belief_intersect(Belief *a, Belief *b)
{
    float result = 0.0f;
    for_each_tile(i)
    {
        result += a->strength[i] * b->strength[i];
    }
    return result;
}

int find_most_likely_match(Target *targets, Belief *b)
{
    float p_max = 0.0f;
    int   i_max = 0;
    for_each_target(i)
    {
        if (targets[i].identified)
            continue; // Skip the ones already identified by this process
        float p = total_belief_intersect(&targets[i].belief, b);
        if (p > p_max)
        {
            p_max = p;
            i_max = i;
        }
    }
    return i_max;
}

float p_or(float *p, int count)
{
    if (count == 0)
        return 0;
    float a = p[0];
    float b = p_or(p+1, count-1);
    float a_or_b = a+b - a*b;
    if (a_or_b > 1.0f) a_or_b = 1.0f;
    if (a_or_b < 0.0f) a_or_b = 0.0f;
    return a_or_b;
}

void belief_strengthen(Belief *a, Belief *b,
                       Belief *result)
{
    float k_forget = 0.4f;
    for_each_tile(i)
    {
        float sa = a->strength[i];
        float sb = b->strength[i];
        float sr = k_forget * sa + (1.0f - k_forget) * sb;
        result->strength[i] = sr;
    }
}

float tile_to_float(int x)
{
    return (float)x + 0.5f;
}

void tile_to_float(int x, int y,
                   float *out_x, float *out_y)
{
    *out_x = tile_to_float(x);
    *out_y = tile_to_float(y);
}

void hidden_belief_update(Belief *a,
                          float time,
                          float time_since_last_seen,
                          float last_seen_x,
                          float last_seen_y,
                          float last_seen_q)
{
    float sx = 8.0f;
    float sy = 1.4f;
    // TODO: Actually formalize coordinate system
    // and corresponding transformations
    float sq = -last_seen_q + 3.1415926f / 2.0f;
    float c = cos(sq);
    float s = sin(sq);
    float s2 = sin(2*sq);
    float a11 = 0.5f * (c*c/sx + s*s/sy);
    float a12 = 0.25f * (-s2/sx + s2/sy);
    float a22 = 0.5f * (s*s/sx + c*c/sy);

    belief_clear(a, 0.0025f);
    for (int y = 0; y < Tile_Dim; y++)
    {
        for (int x = 0; x < Tile_Dim; x++)
        {
            float x1 = tile_to_float(x) - last_seen_x;
            float x2 = tile_to_float(y) - last_seen_y;
            float f = exp(-(a11*x1*x1 + 2.0f*a12*x1*x2 + a22*x2*x2));
            belief_set(a, x, y, f);
        }
    }
}

int main(int argc, char **argv)
{
    sim_init_msgs(false);

    Ai ai;
    ai.time = 0.0f;
    ai.drone_x = 0.0f;
    ai.drone_y = 0.0f;

    for_each_target(i)
    {
        belief_clear(&ai.targets[i].belief, 0.0025f);
        ai.targets[i].visible = false;
        ai.targets[i].vision_index = 0;

        ai.targets[i].last_seen_time = 0.0f;
        ai.targets[i].last_seen_x = 0.0f;
        ai.targets[i].last_seen_y = 0.0f;
        ai.targets[i].last_seen_q = 0.0f;
        ai.targets[i].time_until_reverse = 0.0f;
    }

    int running = 1;
    while (running)
    {
        // Attempt to get the latest state data from simulator
        sim_State state = {};
        sim_recv_state(&state);

        tile_to_float(state.drone_tile_x,
                      state.drone_tile_y,
                      &ai.drone_x,
                      &ai.drone_y);

        float delta_time = state.elapsed_sim_time - ai.time;
        ai.time = state.elapsed_sim_time;

        printf("t = %.2f\ndt = %.2f\n", ai.time, delta_time);

        for_each_target(i)
        {
            ai.targets[i].visible = 0;
            ai.targets[i].identified = 0;
        }

        for_each_target(i)
        {
            if (!state.target_in_view[i])
                continue;
            Belief belief;
            int x = state.drone_tile_x;
            int y = state.drone_tile_y;

            // Since there may be bias in the drone position
            // estimate, the belief will be like a gauss kernel
            // applied about (x, y)
            belief_clear(&belief, 0.0f);
            belief_set(&belief, x,   y,   1.0f);
            belief_set(&belief, x-1, y,   0.35f);
            belief_set(&belief, x+1, y,   0.35f);
            belief_set(&belief, x,   y-1, 0.35f);
            belief_set(&belief, x,   y+1, 0.35f);
            belief_set(&belief, x-1, y-1, 0.12f);
            belief_set(&belief, x+1, y-1, 0.12f);
            belief_set(&belief, x+1, y+1, 0.12f);
            belief_set(&belief, x-1, y+1, 0.12f);

            int match = find_most_likely_match(ai.targets, &belief);
            belief_strengthen(&ai.targets[match].belief, &belief,
                              &ai.targets[match].belief);
            ai.targets[match].identified = true;
            ai.targets[match].visible = true;
            ai.targets[match].vision_index = i;
            ai.targets[match].last_seen_time = ai.time;
            ai.targets[match].last_seen_x = tile_to_float(x);
            ai.targets[match].last_seen_y = tile_to_float(y);
            ai.targets[match].last_seen_q = state.target_q[i];

            if (state.target_reversing[i])
                ai.targets[match].time_until_reverse = 20.0f;
            ai.targets[match].time_until_reverse -= delta_time;

            printf("saw %d, looks like %d, %.2f\n", i, match,
                   ai.targets[match].last_seen_q);
        }

        for_each_target(i)
        {
            if (!ai.targets[i].visible)
            {
                printf("update %d\n", i);
                hidden_belief_update(&ai.targets[i].belief,
                                     ai.time,
                                     ai.targets[i].last_seen_time,
                                     ai.targets[i].last_seen_x,
                                     ai.targets[i].last_seen_y,
                                     ai.targets[i].last_seen_q);
            }
        }

        printf("...............\n\n");

        debug_UserData userdata;
        for (int yi = 0; yi < 20; yi++)
        for (int xi = 0; xi < 20; xi++)
        {
            float ps[Num_Targets];
            for_each_target(i)
            {
                ps[i] = belief_at(&ai.targets[i].belief, xi, yi);
            }
            float p = p_or(ps, Num_Targets);
            userdata.strength[yi][xi] = (unsigned char)(p * 255.0f);
        }

        sim_Command cmd;
        cmd.type = sim_CommandType_Search;
        cmd.x = 2.0f;
        cmd.y = 2.0f;
        cmd.userdata = userdata;
        sim_send_cmd(&cmd);
    }

    udp_close();
}

// Writing this I ran into an interesting problem that
// I had not considered. When storing an array (strength[20][20])
// in the network packet struct, a simply cast-to-char-pointer
// -and-memcpy _will_ cause bugs. I don't quite understand how
// this works, but here's what caused a problem:

// 1. debug_Userdata userdata;
// 2. for (...) userdata.strength[i] = ...;
// 3. sim_Command cmd;
// 4. cmd.userdata = userdata;
// 5. ...
// 6. sim_send_cmd(&cmd);

// In particular, I think line 4 is what caused the issue.
// Even though strength is stored as a static array in the
// struct (i.e. taking sizeof(sim_Command) does include
// the number of elements in the array), assigning userdata
// does not copy the elements?
// How does C++ work here?

// Does the pointer change to point to the external userdata,
// but the remaining elements ... what?

// printf("%x %x\n", &userdata, &cmd.userdata);
// prints different addresses. So the pointer does not change...
// Wtf then?

// Oh... It is the default struct copy assignment?
// Does it actually memcpy? In that case why did stuff fail?

// Well, testing it again it actually works. So what the heck.
