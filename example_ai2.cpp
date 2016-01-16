#define SIM_CLIENT_CODE
#include "simulator.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#define Tile_Dim 20
#define Num_Sims 512
#define for_each_tile(it)    for (int it = 0; it < Tile_Dim*Tile_Dim; it++)
#define for_each_tilerow(it) for (int it = 0; it < Tile_Dim;          it++)
#define for_each_target(it)  for (int it = 0; it < Num_Targets;       it++)
#define for_each_sim(it)     for (int it = 0; it < Num_Sims;          it++)
#define Two_Pi 6.28318530718f
#define One_Pi 3.14159265359f

#define Reverse_Interval 20.5f
#define Target_Speed 0.33f
#define Target_Collision_Radius 0.7f
#define Target_Collision_Radius2 (Target_Collision_Radius*Target_Collision_Radius)
// Can we query this radius from Perception?
#define Target_Detection_Radius 2.0f
#define Sigma_Single_Sample 1.0f

float clamp(float x, float low, float high)
{
    if (x < low) x = low;
    if (x > high) x = high;
    return x;
}

struct Simulation
{
    float x[Num_Targets];
    float y[Num_Targets];
    float q[Num_Targets];
    float t[Num_Targets];
};

void sim_tick(Simulation *sim, float dt)
{
    for_each_target(target)
    {
        float x = sim->x[target];
        float y = sim->y[target];
        float q = sim->q[target];
        float t = sim->t[target];

        for_each_target(other)
        {
            if (target == other)
                continue;
            float dx = x - sim->x[other];
            float dy = y - sim->y[other];
            float dp = dx*dx + dy*dy;
            if (dp < Target_Collision_Radius2)
            {
                q += One_Pi;
                sim->q[other] += One_Pi;
            }
        }

        float vx = Target_Speed * cos(q);
        float vy = Target_Speed * sin(q);
        x += vx * dt;
        y += vy * dt;
        t -= dt;
        if (t < 0.0f)
        {
            t = Reverse_Interval;
            q += One_Pi;
        }
        sim->x[target] = x;
        sim->y[target] = y;
        sim->q[target] = q;
        sim->t[target] = t;
    }
}

void pdf_set(float *pdf, int x, int y, float p)
{
    assert(x >= 0);
    assert(y >= 0);
    assert(x < Tile_Dim);
    assert(y < Tile_Dim);
    pdf[y * Tile_Dim + x] = p;
}

float pdf_get(float *pdf, int x, int y)
{
    assert(x >= 0);
    assert(y >= 0);
    assert(x < Tile_Dim);
    assert(y < Tile_Dim);
    return pdf[y * Tile_Dim + x];
}

void pdf_gauss(float *pdf,
               float center_x, float center_y,
               float sigma_x, float sigma_y,
               float amplitude)
{
    for_each_tilerow(yi)
    for_each_tilerow(xi)
    {
        float x = (float)xi + 0.5f;
        float y = (float)yi + 0.5f;
        float dx = x - center_x;
        float dy = y - center_y;
        float ex = dx*dx / sigma_x;
        float ey = dy*dy / sigma_y;
        float p0 = pdf_get(pdf, xi, yi);
        float p1 = amplitude * exp(-0.5f*(ex+ey));
        float p2 = p0+p1 - p0*p1;
        pdf_set(pdf, xi, yi, p2);
    }
}

float frand()
{
    return (float)rand() / (float)RAND_MAX;
}

int find_closest_target(float *src_x, float *src_y,
                        float query_x, float query_y)
{
    int best_i = 0;
    float best_dp = 50.0f;
    for_each_target(target)
    {
        float dx = src_x[target] - query_x;
        float dy = src_y[target] - query_y;
        float dp = dx*dx + dy*dy;
        if (dp < best_dp)
        {
            best_dp = dp;
            best_i = target;
        }
    }
    return best_i;
}

int main(int argc, char **argv)
{
    sim_init_msgs(false);

    Simulation sims[Num_Sims];
    for_each_sim(sim)
    {
        for_each_target(target)
        {
            float q = Two_Pi * (float)target / (float)Num_Targets;
            float x = 10.0f + 1.0f * cos(q);
            float y = 10.0f + 1.0f * sin(q);
            sims[sim].x[target] = x;
            sims[sim].y[target] = y;
            sims[sim].q[target] = q;
            sims[sim].t[target] = Reverse_Interval;
        }
    }

    int running = 1;
    float last_elapsed_time = 0.0f;
    while (running)
    {
        // Attempt to get the latest state data from simulator
        sim_State state = {};
        sim_recv_state(&state);

        printf("time: %.2f seconds\n", state.elapsed_sim_time);
        float delta_time = state.elapsed_sim_time - last_elapsed_time;
        float elapsed_time = state.elapsed_sim_time;
        last_elapsed_time = elapsed_time;

        float drone_x = state.drone_x;
        float drone_y = state.drone_y;

        bool any_target_in_view = false;
        int num_seen = 0;
        float seen_x[Num_Targets];
        float seen_y[Num_Targets];
        float seen_q[Num_Targets];
        for_each_target(target)
        {
            if (state.target_in_view[target])
            {
                any_target_in_view = true;
                float x = state.target_rel_x[target] + drone_x;
                float y = state.target_rel_y[target] + drone_y;
                float q = state.target_q[target];
                seen_x[num_seen] = x;
                seen_y[num_seen] = y;
                seen_q[num_seen] = q;
                num_seen++;
            }
        }

        for_each_sim(sim)
        {
            for_each_target(target)
            {
                float x = sims[sim].x[target];
                float y = sims[sim].y[target];
                float dx = x - drone_x;
                float dy = y - drone_y;
                float dp = dx*dx + dy*dy;

                // erase particles that we think is in view
                // but infact aren't.
                if (!any_target_in_view && dp <
                    Target_Detection_Radius*
                    Target_Detection_Radius)
                {
                    // Well, the robot certainly isn't here
                    // So let's just uniformly distribute
                    // where we think it is across the universes
                    float q = Two_Pi * frand();
                    float r = 6.0f * frand();
                    float x = drone_x + Target_Detection_Radius * cos(q);
                    float y = drone_y + Target_Detection_Radius * sin(q);
                    x = clamp(x, 0.0f, 20.0f);
                    y = clamp(y, 0.0f, 20.0f);
                    sims[sim].x[target] = x;
                    sims[sim].y[target] = y;
                    sims[sim].q[target] = q;
                }

                // insert particles that we know is in view
                // but we think aren't.
                // Currently disabled because it does not work well
                #if 1
                else if (any_target_in_view)
                {
                    for (int i = 0; i < num_seen; i++)
                    {
                        float sx = seen_x[i];
                        float sy = seen_y[i];
                        float sq = seen_q[i];
                        int closest = find_closest_target(sims[sim].x,
                                                          sims[sim].y,
                                                          sx, sy);
                        sims[sim].x[closest] = sx;
                        sims[sim].y[closest] = sy;
                        sims[sim].q[closest] = sq;
                        // todo: Estimate reverse timer
                    }
                }
                #endif

                // otherwise, perturb the universe in some
                // random fashion.
                else
                {
                    float n1 = -1.0f + 2.0f * frand();
                    float n2 = -1.0f + 2.0f * frand();
                    float n3 = -1.0f + 2.0f * frand();
                    float n4 = -1.0f + 2.0f * frand();
                    sims[sim].x[target] += n1 * 0.05f * delta_time;
                    sims[sim].y[target] += n2 * 0.05f * delta_time;
                    sims[sim].q[target] += n3 * 0.2f * delta_time;
                }
            }

            // advance
            sim_tick(&sims[sim], delta_time);
        }

        float pdf[Tile_Dim*Tile_Dim];
        for_each_tile(tile)
        {
            pdf[tile] = 0.0f;
        }

        for_each_sim(sim)
        {
            for_each_target(target)
            {
                float x = sims[sim].x[target];
                float y = sims[sim].y[target];
                pdf_gauss(pdf, x, y,
                          Sigma_Single_Sample,
                          Sigma_Single_Sample,
                          1.0f / (float)Num_Sims);
            }
        }

        debug_UserData userdata;
        for_each_tilerow(y)
        for_each_tilerow(x)
        {
            float p = pdf_get(pdf, x, y);
            if (p < 0.0f) p = 0.0f;
            if (p > 1.0f) p = 1.0f;
            userdata.strength[y][x] = (unsigned char)(p * 255.0f);
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
