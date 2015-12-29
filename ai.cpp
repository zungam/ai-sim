#define SIM_CLIENT_CODE
#include "simulator.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#define Tile_Dim 20
#define Num_Sims 1024
#define for_each_tile(it)    for (int it = 0; it < Tile_Dim*Tile_Dim; it++)
#define for_each_tilerow(it) for (int it = 0; it < Tile_Dim;          it++)
#define for_each_target(it)  for (int it = 0; it < Num_Targets;       it++)
#define for_each_sim(it)     for (int it = 0; it < Num_Sims;          it++)
#define Two_Pi 6.28318530718f
#define One_Pi 3.14159265359f

#define Reverse_Interval 20.0f
#define Target_Speed 0.33f
#define Target_Collision_Radius 0.7f
#define Target_Collision_Radius2 (Target_Collision_Radius*Target_Collision_Radius)
// Can we query this radius from Perception?
#define Target_Detection_Radius 3.0f
#define Target_Detection_Radius2 (Target_Detection_Radius*Target_Detection_Radius)
#define Sigma_Single_Sample 1.0f

// Each particle is assigned a "confidence" level,
// which decreases over time to indicate that the
// particle is not very useful as an estimate.
// When we observe a real ground robot, we pick
// the particle with lowest confidence in each
// universe, and assign it to this observation
// with high confidence.
#define Confidence_Decrease_Rate 0.05f
#define Lowest_Confidence_Level (1.0f / (float)Num_Sims)

// The particles of each universe are perturbed randomly
// each tick. This is to simulate events like undetected
// collisions and random turns, as well as any uncertainty
// in our own position estimate that we used to estimate the
// initial position of the particle. The amount of perturbation
// increases with lower confidence, from *_Min to *_Max.
#define Perturb_X_Min 0.02f
#define Perturb_X_Max 0.20f
#define Perturb_Y_Min 0.02f
#define Perturb_Y_Max 0.20f
#define Perturb_Q_Min 0.02f
#define Perturb_Q_Max 0.20f

float clamp(float x, float low, float high)
{
    if (x < low) x = low;
    if (x > high) x = high;
    return x;
}

float lerp(float t, float low, float high)
{
    return low + (high - low) * t;
}

struct Simulation
{
    float x[Num_Targets];
    float y[Num_Targets];
    float q[Num_Targets];
    float t[Num_Targets];
    float confidence[Num_Targets];
};

float update_confidence(float c0, float dt)
{
    float r = Confidence_Decrease_Rate;
    float b = Lowest_Confidence_Level;
    float c1 = (c0 - b) * exp(-r*dt) + b;
    return c1;
}

void sim_tick(Simulation *sim, float dt)
{
    for_each_target(target)
    {
        float x = sim->x[target];
        float y = sim->y[target];
        float q = sim->q[target];
        float t = sim->t[target];
        float c = sim->confidence[target];
        c = update_confidence(c, dt);

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
        sim->confidence[target] = c;
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

float frand_0_1()
{
    return (float)rand() / (float)RAND_MAX;
}

float frand_1_1()
{
    return -1.0f + 2.0f * frand_0_1();
}

int find_lowest_confidence(float *confidence)
{
    int min_i = 0;
    float min_c = confidence[0];
    for_each_target(target)
    {
        float c = confidence[target];
        if (c < min_c)
        {
            min_c = c;
            min_i = target;
        }
    }
    return min_i;
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
            sims[sim].confidence[target] = Lowest_Confidence_Level;
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

        float drone_x;
        float drone_y;
        tile_to_world(state.drone_tile_x,
                      state.drone_tile_y,
                      &drone_x, &drone_y);

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
            int num_should_be_seen = num_seen;
            for_each_target(target)
            {
                float x = sims[sim].x[target];
                float y = sims[sim].y[target];
                float dx = x - drone_x;
                float dy = y - drone_y;
                float dp = dx*dx + dy*dy;
                bool  visible = dp < Target_Detection_Radius2;

                // erase particles that we think is in view
                // but infact aren't.
                if (!any_target_in_view && visible)
                {
                    // Well, the robot certainly isn't here
                    // So let's just uniformly distribute
                    // where we think it is across the universes
                    float q = Two_Pi * frand_0_1();
                    float r = 10.0f * frand_0_1();
                    float x = drone_x + Target_Detection_Radius * cos(q);
                    float y = drone_y + Target_Detection_Radius * sin(q);
                    x = clamp(x, 0.0f, 20.0f);
                    y = clamp(y, 0.0f, 20.0f);
                    sims[sim].x[target] = x;
                    sims[sim].y[target] = y;
                    sims[sim].q[target] = Two_Pi * frand_0_1();
                }

                else if (num_should_be_seen > 0 && visible)
                {
                    sims[sim].confidence[target] = 1.0f;
                    num_should_be_seen--;
                }

                // otherwise, perturb the universe in some
                // random fashion. The amount of perturbation
                // will increase proportionally to how little
                // confidence we have any given particle of the
                // universe.
                else
                {
                    float c = sims[sim].confidence[target];
                    float px = lerp(1.0f - c, Perturb_X_Min, Perturb_X_Max);
                    float py = lerp(1.0f - c, Perturb_Y_Min, Perturb_Y_Max);
                    float pq = lerp(1.0f - c, Perturb_Q_Min, Perturb_Q_Max);
                    sims[sim].x[target] += frand_1_1() * px * delta_time;
                    sims[sim].y[target] += frand_1_1() * py * delta_time;
                    sims[sim].q[target] += frand_1_1() * pq * delta_time;
                }
            }

            while (num_should_be_seen > 0)
            {
                // pick some particles with lowest confidence and
                // place them here
                int i = find_lowest_confidence(sims[sim].confidence);
                int seen = num_seen - num_should_be_seen;
                sims[sim].x[i] = seen_x[seen];
                sims[sim].y[i] = seen_y[seen];
                sims[sim].q[i] = seen_q[seen];
                sims[sim].confidence[i] = 1.0f;
                // todo: Estimate reverse timer
                num_should_be_seen--;
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
