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
#define SPEED_MULTIPLIER 1

#ifndef PI
#define PI 3.14159265359f
#endif

#ifndef TWO_PI
#define TWO_PI 6.28318530718f
#endif

struct Color { float r, g, b, a; };
global Color _line_color;
global float _line_scale_x;
global float _line_scale_y;

struct sim_Robot
{
    robot_State state;
    robot_Internal internal;
    robot_Action action;

    float x;  // x-position relative center of floor (m)
    float y;  // y-position relative center of floor (m)
    float L;  // distance between wheels (m)
    float vl; // left-wheel speed (m/s)
    float vr; // right-wheel speed (m/s)
    float q;  // angle relative x-axis (radians)

    float tangent_x;
    float tangent_y;
    float forward_x;
    float forward_y;
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

    float x; // x-position relative center of floor (m)
    float y; // y-position relative center of floor (m)

    float xr; // target x-position (m)
    float yr; // target y-position (m)
};

#define Num_Robots (Num_Targets + Num_Obstacles)
static sim_Robot robots[Num_Robots];
static sim_Robot *targets;
static sim_Robot *obstacles;
static sim_Drone drone;

void
set_color(float r, float g, float b, float a)
{
    Color color = { r, g, b, a };
    _line_color = color;
}

void
set_scale(float x, float y)
{
    _line_scale_x = x;
    _line_scale_y = y;
}

void
draw_line(float x1, float y1, float x2, float y2)
{
    glColor4f(_line_color.r, _line_color.g, _line_color.b, _line_color.a);
    glVertex2f(x1 / _line_scale_x, y1 / _line_scale_y);
    glColor4f(_line_color.r, _line_color.g, _line_color.b, _line_color.a);
    glVertex2f(x2 / _line_scale_x, y2 / _line_scale_y);
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

void robot_observe_state(sim_Robot *robot,
                         float *out_x,
                         float *out_y,
                         float *out_vx,
                         float *out_vy,
                         float *out_q)
{
    // TODO: Add statistical uncertainty
    // TODO: Might need multiple states to
    // fully simulate statistical properties.
    float v = 0.5f * (robot->vl + robot->vr);
    *out_x = robot->x;
    *out_y = robot->y;
    *out_vx = -v * sin(robot->q);
    *out_vy =  v * cos(robot->q);
    *out_q = robot->q;
}

void drone_integrate(sim_Drone *drone, float dt)
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
    drone->x = c1 * drone->x1 + c2 * drone->x2;

    // The dynamics from reference y to y is the same as above.
    float Dy1 = a11 * drone->y1 + a12 * drone->y2 + drone->yr;
    float Dy2 = a21 * drone->y1 + a22 * drone->y2;
    drone->y1 += Dy1 * dt;
    drone->y2 += Dy2 * dt;
    drone->y = c1 * drone->y1 + c2 * drone->y2;
}

void robot_integrate(sim_Robot *robot, float dt)
{
    // TODO: Proper integration to avoid pulsating radii
    float v = 0.5f * (robot->vl + robot->vr);
    float w = (robot->vr - robot->vl) / (robot->L*0.5f);
    // robot->x += -v * sin(robot->q) * dt;
    // robot->y +=  v * cos(robot->q) * dt;
    if (abs(w) < 0.001f)
    {
        robot->x += -v*sin(robot->q)*dt;
        robot->y +=  v*cos(robot->q)*dt;
    }
    else
    {
        robot->x += (v / w) * (cos(robot->q + w*dt) - cos(robot->q));
        robot->y += (v / w) * (sin(robot->q + w*dt) - sin(robot->q));
    }
    robot->q += w * dt;

    while (robot->q >= TWO_PI)
        robot->q -= TWO_PI;

    robot->tangent_x = cos(robot->q);
    robot->tangent_y = sin(robot->q);
    robot->forward_x = -robot->tangent_y;
    robot->forward_y =  robot->tangent_x;
}

void advance_state(float dt)
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
            if (i == n)
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
                float L = sqrt(dx*dx + dy*dy);
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

    for (u32 i = 0; i < Num_Robots; i++)
    {
        robot_integrate(&robots[i], dt);
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
}

void draw_robot(sim_Robot *r)
{
    draw_circle(r->x, r->y, r->L * 0.5f);
    draw_line(r->x - r->tangent_x * r->L * 0.5f,
              r->y - r->tangent_y * r->L * 0.5f,
              r->x + r->tangent_x * r->L * 0.5f,
              r->y + r->tangent_y * r->L * 0.5f);
    draw_line(r->x, r->y,
              r->x + r->forward_x * 0.5f,
              r->y + r->forward_y * 0.5f);
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
    printf("--load\n");
}

void
sim_init(VideoMode mode)
{
    set_color(1.0f, 0.5f, 0.3f, 1.0f);
    printf("--init\n");
    printf("width: %d\n", mode.width);
    printf("height: %d\n", mode.height);
    printf("gl_major: %d\n", mode.gl_major);
    printf("gl_minor: %d\n", mode.gl_minor);
    printf("double_buffer: %d\n", mode.double_buffer);
    printf("depth_bits: %d\n", mode.depth_bits);
    printf("stencil_bits: %d\n", mode.stencil_bits);
    printf("multisamples: %d\n", mode.multisamples);
    printf("swap_interval: %d\n", mode.swap_interval);

    sim_init_msgs(true);

    targets = robots;
    obstacles = robots + Num_Targets;

    for (u32 i = 0; i < Num_Targets; i++)
    {
        float t = TWO_PI * i / (float)(Num_Targets);

        sim_Robot robot = {};
        robot.L = 0.5f;
        robot.x = -1.0f * sin(t);
        robot.y = 1.0f * cos(t);
        robot.q = t;
        robot.internal.initialized = false;
        robot.state = Robot_Start;

        targets[i] = robot;
    }

    for (u32 i = 0; i < Num_Obstacles; i++)
    {
        float t = TWO_PI * i / (float)(Num_Obstacles);

        sim_Robot robot = {};
        robot.L = 0.5f;
        robot.x = -5.0f * sin(t);
        robot.y = 5.0f * cos(t);
        robot.q = t + PI / 2.0f;
        robot.internal.initialized = false;
        robot.state = Robot_Start;

        obstacles[i] = robot;
    }

    // These parameters were chosen somewhat willy-nilly
    // TODO: Record some videos of our drone and fit the
    // model parameters, so that we atleast get the time
    // constants of the system right. We don't really care
    // too much about the exact dynamics, but the timing
    // information is pretty important.
    drone.zeta = 0.707f;
    drone.w = 0.32f;
    drone.a = -0.2f;
    drone.x = 0.0f;
    drone.x1 = 0.0f;
    drone.x2 = 0.0f;
    drone.y = 0.0f;
    drone.y1 = 0.0f;
    drone.y2 = 0.0f;
    drone.xr = 0.0f;
    drone.yr = 0.0f;
}

void
sim_tick(VideoMode mode, float t, float dt)
{
    for (u32 i = 0; i < SPEED_MULTIPLIER; i++)
    {
        drone_integrate(&drone, dt);
        advance_state(dt);
    }

    sim_Command cmd = {};
    if (sim_recv_cmd(&cmd))
    {
        if (cmd.type == sim_CommandType_Goto)
        {
            drone.xr = cmd.x;
            drone.yr = cmd.y;
        }
        else if (cmd.type == sim_CommandType_Search)
        {
            // ?
        }
    }

    // Send a test package once per second
    persist r32 udp_send_timer = 1.0f;
    udp_send_timer -= dt;
    if (udp_send_timer <= 0.0f)
    {
        udp_send_timer = 1.0f;
        sim_State state = {};
        state.elapsed_sim_time = t;

        state.drone_x = drone.x;
        state.drone_y = drone.y;
        state.drone_z = 0.0f;

        for (u32 i = 0; i < Num_Targets; i++)
        {
            robot_observe_state(&targets[i],
                                &state.target_x[i],
                                &state.target_y[i],
                                &state.target_vx[i],
                                &state.target_vy[i],
                                &state.target_q[i]);
        }

        for (u32 i = 0; i < Num_Obstacles; i++)
        {
            robot_observe_state(&targets[i],
                                &state.obstacle_x[i],
                                &state.obstacle_y[i],
                                &state.obstacle_vx[i],
                                &state.obstacle_vy[i],
                                &state.obstacle_q[i]);
        }

        sim_send_state(&state);
    }

    r32 a = mode.width/(r32)mode.height;
    set_scale(a*12.0f, 12.0f);
    glViewport(0, 0, mode.width, mode.height);
    glClearColor(0.03f, 0.02f, 0.01f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glLineWidth(2.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBegin(GL_LINES);
    {
        set_color(1.0f, 0.95f, 0.7f, 0.45f);

        // draw grid
        set_color(1.0f, 0.95f, 0.7f, 0.45f);
        for (u32 i = 0; i <= 20; i++)
        {
            float x = (-1.0f + 2.0f * i / 20.0f) * 10.0f;
            draw_line(x, -10.0f, x, +10.0f);
            draw_line(-10.0f, x, +10.0f, x);
        }

        // draw targets
        set_color(1.0f, 0.35f, 0.11f, 1.0f);
        for (u32 i = 0; i < Num_Targets; i++)
            draw_robot(&targets[i]);

        // draw obstacles
        set_color(1.0f, 0.9f, 0.1f, 1.0f);
        for (u32 i = 0; i < Num_Obstacles; i++)
            draw_robot(&obstacles[i]);

        // draw drone
        set_color(0.1f, 1.0f, 1.0f, 1.0f);
        draw_line(drone.x - 0.5f, drone.y,
                  drone.x + 0.5f, drone.y);
        draw_line(drone.x, drone.y - 0.5f,
                  drone.x, drone.y + 0.5f);

        // draw drone goto
        set_color(0.2f, 1.0f, 0.2f, 0.5f);
        draw_circle(drone.xr, drone.yr, 0.25f);
    }
    glEnd();
}
