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
    float time_until_reverse;

    float estimated_x;
    float estimated_y;
    float estimated_q;

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

// Modifies a to include b.
void belief_strengthen(Belief *a, Belief *b, float forget)
{
    for_each_tile(i)
    {
        float sa = a->strength[i];
        float sb = b->strength[i];
        float sr = (1.0f - forget) * sa + forget * sb;
        a->strength[i] = sr;
    }
}

void belief_weaken(Belief *a, Belief *b, float forget)
{
    for_each_tile(i)
    {
        float sa = a->strength[i];
        float sb = b->strength[i];
        float sr = (1.0f - forget) * sa - forget * sb;
        if (sr < 0.0f)
            sr = 0.0f;
        a->strength[i] = sr;
    }
}

void belief_apply_gaussian(Belief *a,
                           float sigma_x,
                           float sigma_y,
                           float theta,
                           float center_x,
                           float center_y)
{
    float c = cos(-theta);
    float s = sin(-theta);
    float s2 = sin(-2.0f*theta);
    float a11 = 0.5f * (c*c/sigma_x + s*s/sigma_y);
    float a12 = 0.25f * (-s2/sigma_x + s2/sigma_y);
    float a22 = 0.5f * (s*s/sigma_x + c*c/sigma_y);

    for (int y = 0; y < Tile_Dim; y++)
    {
        for (int x = 0; x < Tile_Dim; x++)
        {
            float x1, x2;
            tile_to_world(x, y, &x1, &x2);
            x1 = x1 - center_x;
            x2 = x2 - center_y;
            float p = exp(-(a11*x1*x1 + 2.0f*a12*x1*x2 + a22*x2*x2));
            belief_set(a, x, y, p);
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
        float q = 2.0f * 3.1415926f * i / (float)(Num_Targets);
        belief_clear(&ai.targets[i].belief, 0.0025f);
        belief_apply_gaussian(&ai.targets[i].belief,
                              14.0f, 1.4f, q,
                              10.0f + cos(q) * 3.3f,
                              10.0f + sin(q) * 3.3f);
        ai.targets[i].visible = false;
        ai.targets[i].vision_index = 0;

        ai.targets[i].last_seen_time = -1.0f;
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

        tile_to_world(state.drone_tile_x,
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
            float drone_x, drone_y;
            tile_to_world(state.drone_tile_x,
                          state.drone_tile_y,
                          &drone_x, &drone_y);
            float rel_x = state.target_rel_x[i];
            float rel_y = state.target_rel_y[i];
            float world_x = drone_x + rel_x;
            float world_y = drone_y + rel_y;
            int x, y;
            world_to_tile(world_x, world_y, &x, &y);

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
            belief_strengthen(&ai.targets[match].belief, &belief, 0.3f);
            ai.targets[match].identified = true;
            ai.targets[match].visible = true;
            ai.targets[match].vision_index = i;
            ai.targets[match].last_seen_time = ai.time;
            ai.targets[match].last_seen_x = world_x;
            ai.targets[match].last_seen_y = world_y;
            ai.targets[match].last_seen_q = state.target_q[i];
            ai.targets[match].estimated_x = world_x;
            ai.targets[match].estimated_y = world_y;
            ai.targets[match].estimated_q = state.target_q[i];

            // TODO: Reverse state is active whether it is
            // a timed reverse or because it collided. Figure
            // out how to cope with this. Until then, assume
            // global timing.
            // if (state.target_reversing[i])
            //     ai.targets[match].time_until_reverse = 20.0f;

            printf("saw %d. might reverse in %.2f sec\n", i,
                   ai.targets[match].time_until_reverse);
        }

        for_each_target(i)
        {
            ai.targets[i].time_until_reverse -= delta_time;
            if (ai.targets[i].time_until_reverse <= 0.0f)
            {
                ai.targets[i].time_until_reverse = 20.0f;
                ai.targets[i].estimated_q += 3.1415926f;
            }

            if (!ai.targets[i].visible && ai.targets[i].last_seen_time >= 0.0f)
            {
                float x = ai.targets[i].estimated_x;
                float y = ai.targets[i].estimated_y;
                float q = ai.targets[i].estimated_q;
                Belief belief;
                belief_clear(&belief, 0.0025f);
                float alpha = ai.time - ai.targets[i].last_seen_time;
                float e = exp(-0.0025f * alpha);
                float sigma_x = 6.0f*e + 12.0f*(1.0f-e);
                float sigma_y = 2.5f*e + 15.0f*(1.0f-e);
                belief_apply_gaussian(&belief, sigma_x, sigma_y, q, x, y);
                belief_strengthen(&ai.targets[i].belief, &belief, 0.34f);

                belief_clear(&belief, 0.0f);
                belief_apply_gaussian(&belief, 4.0f, 4.0f, 0.0f,
                                      ai.drone_x, ai.drone_y);
                belief_weaken(&ai.targets[i].belief, &belief, 0.15f);
                ai.targets[i].estimated_x += 0.33f * cos(q) * delta_time;
                ai.targets[i].estimated_y += 0.33f * sin(q) * delta_time;
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
