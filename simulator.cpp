#include "platform.h"
#include "simulator.h"
#include "sim_robot.cpp"
#include "SDL_opengl.h"
#include <stdio.h>
#include <math.h>

// This is purely for debugging because the
// robots are moving so darn slow. Helps me
// visualize the dynamics better by speeding
// it up.
#define SPEED_MULTIPLIER 4
#define STATE_SEND_INTERVAL 0.25f

// Apply stochastic bias to observed drone position
// #define ENABLE_BIAS

// Respond to commands received from AI
#define ENABLE_AI_CMD

// Visualize probability density field
// #define ENABLE_AI_USERDATA

#ifndef PI
#define PI 3.14159265359f
#endif

#ifndef TWO_PI
#define TWO_PI 6.28318530718f
#endif

struct Color { float r, g, b, a; };
global Color _line_color;
global float _ndc_scale_x;
global float _ndc_scale_y;

void
world_to_ndc(float x_world, float y_world,
             float *x_ndc, float *y_ndc)
{
    float x = (x_world - 10.0f) * _ndc_scale_x;
    float y = (y_world - 10.0f) * _ndc_scale_y;
    *x_ndc = x;
    *y_ndc = y;
}

struct sim_Robot
{
    robot_State state;
    robot_Internal internal;
    robot_Action action;

    float x;  // x position in world frame (m)
    float y;  // y position in world frame (m)
    float L;  // distance between wheels (m)
    float vl; // left-wheel speed (m/s)
    float vr; // right-wheel speed (m/s)
    float q;  // angle relative x-axis (radians)
    float height; // height of body from ground (m).
                  // Includes cylinder for the obstacles.

    float tangent_x;
    float tangent_y;
    float forward_x;
    float forward_y;

    bool removed;
};

struct sim_Drone
{
    // Parameters describing the dynamics from
    // reference position to position:
    float zeta; // relative damping factor
    float w;    // natural frequency
    float a;    // backshoot period

    // These are "virtual" states that are only
    // needed to realize the transfer functions.
    float x1;
    float x2;
    float y1;
    float y2;
    float z1;
    float z2;

    float x; // x-position in world frame (m)
    float y; // y-position in world frame (m)
    float z; // height from ground (m)

    float xr; // target x-position (m)
    float yr; // target y-position (m)
    float zr; // target height (m)

    float v_max; // maximum speed in x and y (seperately)

    sim_Command cmd; // current active command
    bool cmd_complete;

    // These variables are strictly used when observing
    // the drone state, and do not affect the internal
    // simulation:

    // The drone's position measurement has statistical
    // uncertainty. I'm going to assume that our Perception
    // group has managed to give us an estimate which does
    // not have high frequency noise on top. I will however
    // assume that there may be significant bias in the
    // estimate, in the order of a couple of tiles.
    // I'll assume that the bias in one of the coordinates
    // may be reset whenever the drone sees an edge.
    float bias_x;
    float bias_y;
};

#define Num_Robots (Num_Targets + Num_Obstacles)
static sim_Robot robots[Num_Robots];
static sim_Robot *targets;
static sim_Robot *obstacles;
static sim_Drone drone;

void
set_color(float r, float g, float b, float a)
{
    glColor4f(r, g, b, a);
}

void
draw_line(float x1, float y1, float x2, float y2)
{
    float x1n, y1n, x2n, y2n;
    world_to_ndc(x1, y1, &x1n, &y1n);
    world_to_ndc(x2, y2, &x2n, &y2n);
    glVertex2f(x1n, y1n);
    glVertex2f(x2n, y2n);
}

void
fill_square(float x1, float y1, float x2, float y2)
{
    float x1n, y1n, x2n, y2n;
    world_to_ndc(x1, y1, &x1n, &y1n);
    world_to_ndc(x2, y2, &x2n, &y2n);
    glVertex2f(x1n, y1n);
    glVertex2f(x2n, y1n);
    glVertex2f(x2n, y2n);
    glVertex2f(x2n, y2n);
    glVertex2f(x1n, y2n);
    glVertex2f(x1n, y1n);
}

void
fill_circle(float x, float y, float r)
{
    float xn, yn;
    world_to_ndc(x, y, &xn, &yn);
    u32 n = 32;
    for (u32 i = 0; i < n; i++)
    {
        float t1 = TWO_PI * i / (r32)n;
        float t2 = TWO_PI * (i + 1) / (r32)n;
        float x1 = x + r*cos(t1);
        float y1 = y + r*sin(t1);
        float x2 = x + r*cos(t2);
        float y2 = y + r*sin(t2);
        float x1n, y1n, x2n, y2n;
        world_to_ndc(x1, y1, &x1n, &y1n);
        world_to_ndc(x2, y2, &x2n, &y2n);
        glVertex2f(xn, yn);
        glVertex2f(x1n, y1n);
        glVertex2f(x2n, y2n);
        glVertex2f(x2n, y2n);
        glVertex2f(xn, yn);
        glVertex2f(xn, yn);
    }
}

void
draw_circle(float x, float y, float r)
{
    u32 n = 32;
    for (u32 i = 0; i < n; i++)
    {
        float t1 = TWO_PI * i / (r32)n;
        float t2 = TWO_PI * (i + 1) / (r32)n;
        float x1 = x + r*cos(t1);
        float y1 = y + r*sin(t1);
        float x2 = x + r*cos(t2);
        float y2 = y + r*sin(t2);
        draw_line(x1, y1, x2, y2);
    }
}

void
robot_observe_state(sim_Robot *robot,
                         float *out_x,
                         float *out_y,
                         float *out_vx,
                         float *out_vy,
                         float *out_q)
{
    float v = 0.5f * (robot->vl + robot->vr);
    *out_x = robot->x;
    *out_y = robot->y;
    *out_vx = -v * sin(robot->q);
    *out_vy =  v * cos(robot->q);
    *out_q = robot->q;
}

float
clamp(float x, float x0, float x1)
{
    if (x < x0) return x0;
    else if (x > x1) return x1;
    else return x;
}

void
drone_integrate(sim_Drone *drone, float dt)
{
    // I've modelled the dynamics from the reference
    // position (xr) to position (x) as
    //    x            w^2 (1 + as)
    //   ---(s) = ----------------------
    //   x_r      s^2 + 2 zeta w s + w^2
    // which represents an oscillatory non-minimum phase
    // system. Because it is oscillatory, the drone might
    // overshoot on its reference. Because it is non-minimum
    // phase, it might backshoot when it receives a new
    // reference. The equations below is a canonical
    // controllable form realization of the transfer function
    // above.
    float x1y1_max = drone->v_max / (drone->w*drone->w);
    float a11 = -2.0f * drone->zeta * drone->w;
    float a12 = -drone->w*drone->w;
    float a21 = 1.0f;
    float a22 = 0.0f;
    float c1 = drone->a * drone->w * drone->w;
    float c2 = drone->w * drone->w;
    float Dx1 = a11 * drone->x1 + a12 * drone->x2 + drone->xr;
    float Dx2 = a21 * drone->x1 + a22 * drone->x2;
    drone->x1 += Dx1 * dt;
    drone->x2 += Dx2 * dt;
    drone->x1 = clamp(drone->x1, -x1y1_max, x1y1_max);
    drone->x = c1 * drone->x1 + c2 * drone->x2;

    // The dynamics from reference y to y is the same as above.
    float Dy1 = a11 * drone->y1 + a12 * drone->y2 + drone->yr;
    float Dy2 = a21 * drone->y1 + a22 * drone->y2;
    drone->y1 += Dy1 * dt;
    drone->y2 += Dy2 * dt;
    drone->y1 = clamp(drone->y1, -x1y1_max, x1y1_max);
    drone->y = c1 * drone->y1 + c2 * drone->y2;

    // And from reference height to height
    float Dz1 = a11 * drone->z1 + a12 * drone->z2 + drone->zr;
    float Dz2 = a21 * drone->z1 + a22 * drone->z2;
    drone->z1 += Dz1 * dt;
    drone->z2 += Dz2 * dt;
    drone->z = c1 * drone->z1 + c2 * drone->z2;
}

void
robot_integrate(sim_Robot *robot, float dt)
{
    float v = 0.5f * (robot->vl + robot->vr);
    float w = (robot->vr - robot->vl) / (robot->L*0.5f);
    #if 0
    // Simplified dynamics
    robot->x += v * cos(robot->q) * dt;
    robot->y += v * sin(robot->q) * dt;
    #else
    if (abs(w) < 0.001f)
    {
        robot->x += v*cos(robot->q)*dt;
        robot->y += v*sin(robot->q)*dt;
    }
    else
    {
        robot->x += (v / w) * (sin(robot->q + w*dt) - sin(robot->q));
        robot->y -= (v / w) * (cos(robot->q + w*dt) - cos(robot->q));
    }
    #endif
    robot->q += w * dt;

    while (robot->q > TWO_PI)
        robot->q -= TWO_PI;
    while (robot->q < 0.0f)
        robot->q += TWO_PI;

    robot->forward_x = cos(robot->q);
    robot->forward_y = sin(robot->q);
    robot->tangent_x = -sin(robot->q);
    robot->tangent_y = cos(robot->q);
}

float
vector_length(float dx, float dy)
{
    return sqrt(dx*dx + dy*dy);
}

float
compute_camera_view_radius(float height_above_ground)
{
    // Interpolates between 0.5 meters and
    // 3 meters view radius when height goes
    // from 0 to 2.5 meters.
    float h0 = 0.0f;
    float h1 = 3.0f;
    float alpha = (drone.z - h0) / (h1 - h0);
    float view_radius = 0.5f + 2.0f * alpha;
    return view_radius;
}

void
advance_state(float dt)
{
    persist float simulation_time = 0.0f;
    simulation_time += dt;
    struct CollisionInfo
    {
        int hits;
        int bumper_hits;
        float resolve_delta_x;
        float resolve_delta_y;
    };

    robot_Event events[Num_Robots] = {};
    CollisionInfo collision[Num_Robots] = {};

    drone_integrate(&drone, dt);

    for (u32 i = 0; i < Num_Robots; i++)
    {
        if (robots[i].removed) continue;

        events[i].is_run_sig = 0;
        events[i].is_wait_sig = 0;
        events[i].is_top_touch = 0;
        events[i].is_bumper = 0;
        events[i].target_switch_pin = 0;
        events[i].elapsed_sim_time = simulation_time;

        collision[i].hits = 0;
        collision[i].bumper_hits = 0;
        collision[i].resolve_delta_x = 0.0f;
        collision[i].resolve_delta_y = 0.0f;

        switch (robots[i].state)
        {
            case Robot_Start:
            {
                if (i < Num_Targets)
                    events[i].target_switch_pin = 1;
                else
                    events[i].target_switch_pin = 0;
            } break;

            case Robot_TargetWait:
            case Robot_ObstacleWait:
            {
                events[i].is_run_sig = 1;
            } break;
        }

        // Check for collisions and compute the average resolve
        // delta vector. The resolve delta will be used to move
        // the robot away so it no longer collides.
        for (u32 n = 0; n < Num_Robots; n++)
        {
            if (i == n || robots[n].removed)
            {
                continue;
            }
            else
            {
                float x1 = robots[i].x;
                float y1 = robots[i].y;
                float r1 = robots[i].L * 0.5f;
                float x2 = robots[n].x;
                float y2 = robots[n].y;
                float r2 = robots[n].L * 0.5f;
                float dx = x1 - x2;
                float dy = y1 - y2;
                float L = vector_length(dx, dy);
                float intersection = r2 + r1 - L;
                if (intersection > 0.0f)
                {
                    collision[i].hits++;
                    collision[i].resolve_delta_x += (dx / L) * intersection;
                    collision[i].resolve_delta_y += (dy / L) * intersection;

                    // The robot only reacts (in a fsm sense) if the collision
                    // triggers the bumper sensor in front of the robot. (We
                    // still resolve physical collisions anyway, though).
                    // TODO: Determine the angular region that the bumper
                    // sensor covers (I have assumed 180 degrees).
                    bool on_bumper = (dx * robots[i].forward_x +
                                      dy * robots[i].forward_y) <= 0.0f;
                    if (on_bumper)
                        collision[i].bumper_hits++;
                }
            }
        }
        if (collision[i].hits > 0)
        {
            collision[i].resolve_delta_x /= (float)collision[i].hits;
            collision[i].resolve_delta_y /= (float)collision[i].hits;
        }
        if (collision[i].bumper_hits > 0)
            events[i].is_bumper = 1;
    }

    // Ugly command handling
    // Hardcoded in behaviour for top-touching and
    // bumper hitting.
    switch (drone.cmd.type)
    {
        case sim_CommandType_LandOnTopOf:
        {
            if (!drone.cmd_complete)
            {
                drone.xr = targets[drone.cmd.i].x;
                drone.yr = targets[drone.cmd.i].y;
                if (vector_length(drone.xr - drone.x, drone.yr - drone.y)
                    <= targets[drone.cmd.i].L)
                {
                    drone.zr = 0.125f;
                }
                if (drone.z < 0.2f && drone.zr < 0.2f)
                {
                    events[drone.cmd.i].is_top_touch = 1;
                }
                if (targets[drone.cmd.i].action.was_top_touched)
                {
                    drone.zr = 1.5f;
                    drone.cmd_complete = 1;
                }
            }
        } break;
        case sim_CommandType_LandInFrontOf:
        {
            if (!drone.cmd_complete)
            {
                drone.xr = targets[drone.cmd.i].x;
                drone.yr = targets[drone.cmd.i].y;
                if (vector_length(drone.xr - drone.x, drone.yr - drone.y)
                    <= targets[drone.cmd.i].L)
                {
                    drone.zr = 0.05f;
                }
                if (drone.z < 0.1f && drone.zr < 0.1f)
                {
                    events[drone.cmd.i].is_bumper = 1;
                }
                if (targets[drone.cmd.i].action.was_bumped)
                {
                    drone.zr = 1.5f;
                    drone.cmd_complete = 1;
                }
            }
        } break;
        case sim_CommandType_Track:
        {
            // Follow a given robot
            drone.xr = targets[drone.cmd.i].x;
            drone.yr = targets[drone.cmd.i].y;
            drone.zr = 1.5f;
        } break;
        case sim_CommandType_Search:
        {
            // Go to a setpoint and hover high
            drone.xr = drone.cmd.x;
            drone.yr = drone.cmd.y;
            drone.zr = 3.0f;
        } break;
    }

    for (u32 i = 0; i < Num_Robots; i++)
    {
        if (robots[i].removed)
            continue;
        robot_integrate(&robots[i], dt);
        if (robots[i].x < -0.5f ||
            robots[i].x > 20.5f ||
            robots[i].y < -0.5f ||
            robots[i].y > 20.0f)
        {
            robots[i].removed = true;
            robots[i].y = 300.0f; // move far away
        }
        if (collision[i].hits > 0)
        {
            robots[i].x += collision[i].resolve_delta_x * 1.02f;
            robots[i].y += collision[i].resolve_delta_y * 1.02f;
        }
        robots[i].state = robot_fsm(robots[i].state,
                                    &robots[i].internal,
                                    events[i],
                                    &robots[i].action);
        robots[i].vl = robots[i].action.left_wheel;
        robots[i].vr = robots[i].action.right_wheel;
    }

    // Every _drone_bias_timer_ seconds there is a
    // probability of a bias being added to the
    // estimated drone position, in either coordinate,
    // of one grid cell in meters.
    #ifdef ENABLE_BIAS
    persist r32 drone_bias_timer = 2.0f;
    r32 drone_bias_probability = 0.5f;
    drone_bias_timer -= dt;
    if (drone_bias_timer <= 0.0f)
    {
        float p = frand();
        if (p <= drone_bias_probability)
        {
            float random_meters;
            float p2 = frand();
            if (p2 < 0.5f)
                random_meters = -1.0f;
            else
                random_meters = +1.0f;

            if (p <= drone_bias_probability * 0.5f)
                drone.bias_x += random_meters;
            else
                drone.bias_y += random_meters;
        }
        drone_bias_timer = 2.0f;
    }
    #endif

    // Bias is eliminated in atleast one coordinate
    // if the drone detects an edge of the map.
    float drone_view_radius = compute_camera_view_radius(drone.z);
    if (abs(drone.x -  0.0f) < drone_view_radius ||
        abs(drone.x - 20.0f) < drone_view_radius)
        drone.bias_x = 0.0f;
    if (abs(drone.y -  0.0f) < drone_view_radius ||
        abs(drone.y - 20.0f) < drone_view_radius)
        drone.bias_y = 0.0f;
}

void
draw_robot(sim_Robot *r)
{
    if (r->removed) return;
    float x = r->x;
    float y = r->y;
    float l = r->L;
    float q = r->q;
    draw_circle(x, y, l*0.5f);
    draw_line(x, y, x + l*cos(q), y + l*sin(q));
}

void
sim_load(VideoMode *mode)
{
    mode->width = 500;
    mode->height = 500;
    mode->gl_major = 1;
    mode->gl_minor = 5;
    mode->double_buffer = 1;
    mode->depth_bits = 24;
    mode->stencil_bits = 8;
    mode->multisamples = 0;
    mode->swap_interval = 1;
}

void
sim_init(VideoMode mode)
{
    sim_init_msgs(true);

    targets = robots;
    obstacles = robots + Num_Targets;

    for (u32 i = 0; i < Num_Targets; i++)
    {
        sim_Robot robot = {};

        // TODO: Verify the distance between the wheels
        // I've read on iRobot's store that the distance
        // is about 0.34 meters. But that, combined with
        // the +-9mm/s wheel speeds does _not_ make the
        // obstacles rotate about the floor center.
        // So something is wrong, either with my dynamics
        // equations, the measurements, or how I'm interpreting
        // the +-9mm/s number. (Does it actually refer to
        // angular velocity?).
        robot.L = 0.5f;
        robot.height = 0.1f;

        // Spawn each ground robot in a circle
        float t = TWO_PI * i / (float)(Num_Targets);
        robot.x = 10.0f + 1.0f * cos(t);
        robot.y = 10.0f + 1.0f * sin(t);
        robot.q = t;
        robot.internal.initialized = false;
        robot.state = Robot_Start;
        robot.removed = false;

        targets[i] = robot;
    }

    for (u32 i = 0; i < Num_Obstacles; i++)
    {
        float t = TWO_PI * i / (float)(Num_Obstacles);

        sim_Robot robot = {};

        // TODO: Verify the distance between the wheels
        robot.L = 0.5f;

        // See: http://www.aerialroboticscompetition.org/stories/stories4.php
        // Height is actually 1 to 2 meters, but I'm just assuming it's as
        // tall as possible.
        robot.height = 2.0f;

        // The obstacles are also spawned in a circle,
        // but at an initial radius of 5 meters.
        robot.x = 10.0f + 5.0f * cos(t);
        robot.y = 10.0f + 5.0f * sin(t);
        robot.q = t + PI / 2.0f;
        robot.internal.initialized = false;
        robot.state = Robot_Start;
        robot.removed = false;

        obstacles[i] = robot;
    }

    // These parameters were chosen somewhat willy-nilly
    // TODO: Record some videos of our drone and fit the
    // model parameters, so that we atleast get the time
    // constants of the system right. We don't really care
    // too much about the exact dynamics, but the timing
    // information is pretty important.
    drone.zeta = 0.9f;
    drone.w = 3.5f;
    drone.a = -0.03f;
    drone.xr = 10.0f;
    drone.yr = 10.0f;
    drone.x = 10.0f;
    drone.x1 = 0.0f;
    drone.x2 = drone.xr / (drone.w*drone.w);
    drone.y = 10.0f;
    drone.y1 = 0.0f;
    drone.y2 = drone.yr / (drone.w*drone.w);
    drone.z = 1.0f;
    drone.z1 = 0.0f;
    drone.z2 = 0.0f;
    drone.zr = 1.5f;
    drone.v_max = 1.0f;
    drone.cmd.type = sim_CommandType_Search;
    drone.cmd.x = 10.0f;
    drone.cmd.y = 10.0f;
    drone.cmd.i = 0;
    drone.cmd_complete = 0;
    drone.bias_x = 0.0f;
    drone.bias_y = 0.0f;
}

void
sim_tick(VideoMode mode, float app_time, float app_dt)
{
    _ndc_scale_x = (1.0f / 12.0f) * mode.height / (float)mode.width;
    _ndc_scale_y = 1.0f / 12.0f;

    persist debug_UserData userdata;
    persist float elapsed_sim_time = 0.0f;

    float delta_time = app_dt * SPEED_MULTIPLIER;
    elapsed_sim_time += delta_time;

    // Poll for commands sent to the drone
    sim_Command cmd = {};
    if (sim_recv_cmd(&cmd))
    {
        // For now we react to the command in an ad-hoc
        // manner. Later we will want to formalize the
        // drone response to various commands
        #ifdef ENABLE_AI_CMD
        drone.cmd = cmd;
        drone.cmd_complete = 0;
        #endif
        userdata = cmd.userdata;
    }

    const u08 *keys = SDL_GetKeyboardState(0);
    s32 closest_target = 0;
    float closest_target_d = 100.0f;
    for (s32 i = 0; i < Num_Targets; i++)
    {
        float target_x = targets[i].x;
        float target_y = targets[i].y;
        float d = vector_length(target_x - drone.xr, target_y - drone.yr);
        if (d <= closest_target_d)
        {
            closest_target_d = d;
            closest_target = i;
        }
    }
    if (keys[SDL_SCANCODE_LEFT])
    {
        drone.cmd.type = sim_CommandType_Search;
        drone.cmd.x -= 5.0f * app_dt;
    }
    if (keys[SDL_SCANCODE_RIGHT])
    {
        drone.cmd.type = sim_CommandType_Search;
        drone.cmd.x += 5.0f * app_dt;
    }
    if (keys[SDL_SCANCODE_UP])
    {
        drone.cmd.type = sim_CommandType_Search;
        drone.cmd.y += 5.0f * app_dt;
    }
    if (keys[SDL_SCANCODE_DOWN])
    {
        drone.cmd.type = sim_CommandType_Search;
        drone.cmd.y -= 5.0f * app_dt;
    }
    if (keys[SDL_SCANCODE_SPACE])
    {
        drone.cmd.type = sim_CommandType_LandInFrontOf;
        drone.cmd.i = closest_target;
        drone.cmd_complete = 0;
    }
    if (keys[SDL_SCANCODE_L])
    {
        drone.cmd.type = sim_CommandType_LandOnTopOf;
        drone.cmd.i = closest_target;
        drone.cmd_complete = 0;
    }
    if (drone.cmd.type == sim_CommandType_LandInFrontOf ||
        drone.cmd.type == sim_CommandType_LandOnTopOf)
    {
        // Need to synchronize these when keyboard input is used,
        // otherwise we get a jarring jump in the drawn circle
        // when you use keyboard after performing a land command.
        drone.cmd.x = drone.xr;
        drone.cmd.y = drone.yr;
    }

    for (u32 i = 0; i < SPEED_MULTIPLIER; i++)
    {
        advance_state(delta_time / (float)SPEED_MULTIPLIER);
    }

    // How often we send a state measurement
    persist r32 state_send_timer = STATE_SEND_INTERVAL;

    state_send_timer -= app_dt;
    if (state_send_timer <= 0.0f)
    {
        state_send_timer = STATE_SEND_INTERVAL;
        sim_State state = {};
        state.elapsed_sim_time = elapsed_sim_time;

        world_to_tile(drone.x + drone.bias_x,
                      drone.y + drone.bias_y,
                      &state.drone_tile_x,
                      &state.drone_tile_y);
        state.drone_cmd_complete = drone.cmd_complete;

        for (u32 i = 0; i < Num_Targets; i++)
        {
            // I'll assume that we can compute the relative distance
            // between us and a ground robot that is close enough in
            // our view.
            float observation_radius = compute_camera_view_radius(drone.z);
            float dist = vector_length(targets[i].x - drone.x,
                                       targets[i].y - drone.y);
            if (dist < observation_radius)
            {
                state.target_rel_x[i] = targets[i].x - drone.x;
                state.target_rel_y[i] = targets[i].y - drone.y;
                if (targets[i].state == Robot_Reverse)
                    state.target_reversing[i] = true;
                else
                    state.target_reversing[i] = false;
                state.target_q[i] = targets[i].q;
                state.target_in_view[i] = true;
            }
            else
            {
                state.target_in_view[i] = false;
            }
        }

        for (u32 i = 0; i < Num_Obstacles; i++)
        {
            // I'll assume that we have a laser mounted on top
            // of the quad which actually observes the position
            // of the tower robots reasonably.
            state.obstacle_rel_x[i] = obstacles[i].x - drone.x;
            state.obstacle_rel_y[i] = obstacles[i].y - drone.y;
        }

        sim_send_state(&state);
    }

    glViewport(0, 0, mode.width, mode.height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glLineWidth(2.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBegin(GL_TRIANGLES);
    {
        #ifdef ENABLE_AI_USERDATA
        // draw user data (probability field strength)
        for (u32 xi = 0; xi < 20; xi++)
        {
            for (u32 yi = 0; yi < 20; yi++)
            {
                float x1, y1, x2, y2;
                tile_to_world(xi, yi, &x1, &y1);
                tile_to_world(xi+1, yi+1, &x2, &y2);
                float s = (float)userdata.strength[yi][xi] / 255.0f;

                // Colorful color palette
                // float r = 0.5f + 0.5f*sin(6.2832f*(0.3f*s+0.8f));
                // float g = 0.5f + 0.5f*sin(6.2832f*(0.5f*s+0.9f));
                // float b = 0.5f + 0.5f*sin(6.2832f*(0.25f*s+0.3f));

                float r = s * 0.27f;
                float g = s * 0.14f;
                float b = s * 0.20f;
                if (s > 0.5f)
                {
                    r = s * 0.62f;
                    g = s * 0.22f;
                    b = s * 0.18f;
                }
                float a = 1.0f;
                glColor4f(r, g, b, a);
                fill_square(x1, y1, x2, y2);
            }
        }
        #endif

        // draw biased drone position
        // glColor4f(0.46, 0.44, 0.38, 0.35f);
        // int tile_x, tile_y;
        // float draw_x, draw_y;
        // world_to_tile(drone.x + drone.bias_x,
        //               drone.y + drone.bias_y,
        //               &tile_x, &tile_y);
        // tile_to_world(tile_x, tile_y, &draw_x, &draw_y);
        // fill_circle(draw_x+0.5f, draw_y+0.5f, 0.5f);
    }
    glEnd();

    glBegin(GL_LINES);
    {
        // draw grid
        set_color(0.87f, 0.93f, 0.84f, 0.2f);
        for (u32 i = 0; i <= 20; i++)
        {
            float x = (float)i;
            draw_line(x, 0.0f, x, 20.0f);
            draw_line(0.0f, x, 20.0f, x);
        }

        // draw visible region
        set_color(0.87f, 0.93f, 0.84f, 0.5f);
        draw_circle(drone.x, drone.y, compute_camera_view_radius(drone.z));

        // draw green line
        set_color(0.43f, 0.67f, 0.17f, 1.0f);
        draw_line(0.0f, 20.0f, 20.0f, 20.0f);

        // draw targets
        set_color(0.85, 0.83, 0.37, 1.0f);
        for (u32 i = 0; i < Num_Targets; i++)
        {
            // float observation_radius = compute_camera_view_radius(drone.z);
            // float dist = vector_length(targets[i].x - drone.x,
            //                            targets[i].y - drone.y);
            // if (dist < observation_radius)
            draw_robot(&targets[i]);
        }

        // draw obstacles
        set_color(0.43, 0.76, 0.79, 1.0f);
        for (u32 i = 0; i < Num_Obstacles; i++)
            draw_robot(&obstacles[i]);

        // draw drone
        set_color(0.87f, 0.93f, 0.84f, 0.5f);
        draw_line(drone.x - 0.5f, drone.y,
                  drone.x + 0.5f, drone.y);
        draw_line(drone.x, drone.y - 0.5f,
                  drone.x, drone.y + 0.5f);

        // draw drone goto
        set_color(0.87f, 0.93f, 0.84f, 0.5f);
        draw_circle(drone.xr, drone.yr, 0.45f);

        // draw indicators of magnet or bumper activations
        for (u32 i = 0; i < Num_Targets; i++)
        {
            float x = targets[i].x;
            float y = targets[i].y;
            if (targets[i].action.was_bumped)
            {
                set_color(1.0f, 0.3f, 0.1f, 1.0f);
                draw_circle(x, y, 0.5f);
            }
            else if (targets[i].action.was_top_touched)
            {
                set_color(1.0f, 1.0f, 1.0f, 1.0f);
                draw_circle(x, y, 0.5f);
            }
        }
    }
    glEnd();
}
