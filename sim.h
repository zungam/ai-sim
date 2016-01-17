// Simulator for IARC mission 7A - v0.01 - public domain
// http://www.ascendntnu.no/
//
// Written by Simen Haugo in 2016, last updated in 2016
//
// LICENSE
// =======================================================================
//   This software is in the public domain. Where that dedication is not
//   recognized, you are granted a perpetual, irrevocable license to copy,
//   distribute, and modify this file as you see fit.
//
// No warranty for any purpose is expressed or implied by the author (nor
// by Ascend NTNU). Report bugs and send enhancements to the author.
//
// HOW TO COMPILE
// =======================================================================
// This file contains both the header file and the implementation file.
// To compile, insert the following in A SINGLE source file in your project
//
//     #define SIM_IMPLEMENTATION
//     #include "sim.h"
//
// You may otherwise include this file as you would include a traditional
// header file.
//
// HISTORY
// =======================================================================
// 12. January 2016:
// ***********************************************************************
//                        User-interface starts here
// ***********************************************************************

#ifndef SIM_HEADER_INCLUDE
#define SIM_HEADER_INCLUDE
#define Sim_Timestep (1.0f / 60.0f)
#define Sim_Drone_View_Radius (2.5f)
#define Num_Obstacles (4)
#define Num_Targets (10)
#define Num_Robots (Num_Obstacles + Num_Targets)

#include <stdint.h>
typedef float       r32;
typedef uint64_t    u64;
typedef uint32_t    u32;
typedef uint16_t    u16;
typedef uint8_t     u08;
typedef int32_t     s32;
typedef int16_t     s16;
typedef int8_t      s08;

// How many meters the drone should be near to
// a robot before it is considered to be above it.
#define Sim_Drone_Target_Proximity (0.1f)

// How many meters the drone should be near to
// a point in world space before it is considered
// to have reached it.
#define Sim_Drone_Goto_Proximity (0.1f)

// How many seconds it should take for the drone
// to complete a LandOnTopOf command, once it
// enters the target proximity range.
#define Sim_LandOnTopOf_Time (2.0f)

// How many seconds it should take for the drone
// to complete a LandInFrontOf command, once it
// enters the target proximity range.
#define Sim_LandInFrontOf_Time (2.0f)

// How many meters must a target walk beyond
// an edge to be removed
#define Sim_Target_Removal_Margin (0.5f)

// Distance between iRobot wheels in metres
#define Sim_Robot_Wheel_Distance (0.5f)
#define Sim_Target_Init_Radius (1.0f)
#define Sim_Obstacle_Init_Radius (5.0f)

struct sim_Observed_State
{
    float elapsed_time;
    bool  target_in_view[Num_Targets];
    float target_x[Num_Targets];
    float target_y[Num_Targets];
    float target_q[Num_Targets];
    bool  target_removed[Num_Targets];
    float obstacle_x[Num_Obstacles];
    float obstacle_y[Num_Obstacles];
    float obstacle_q[Num_Obstacles];
    float drone_x;
    float drone_y;
    bool  drone_cmd_done;
};

enum sim_CommandType
{
    sim_CommandType_NoCommand = 0,   // continue doing whatever you are doing
    sim_CommandType_LandOnTopOf,     // trigger one 45 deg turn of robot (i)
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

typedef float sim_Time;

// The WORLD-SPACE COORDINATE SYSTEM is defined as follows:
// The red line is considered the x-axis, while the left
// edge connecting the red and green line is considered the
// y-axis. The z-axis is considered out of the screen, following
// the right hand rule of orthogonal frames. The unit is meters.
// (0,0) is the lower-left line intersection, (20,20) is the upper
// right line intersection.
//
//         x=0        x=20
//         y=20       y=20
//          +----------+                    ^ +y
//          |  GREEN   |                    |
//          |          |                    |
//          |          |                    |
//          |   RED    |                    |
//          +----------+                    +-------> +x
//         x=0        x=20
//         y=0        y=0
struct sim_World_Pos
{
    float x;
    float y;
};

// Angle, in radians, relative to the x-axis,
// positive counter-clockwise, wrapped to the
// half-open interval [0, 2xPi).
typedef float sim_World_Angle;

enum robot_State
{
    Robot_Start,
    Robot_ObstacleWait,
    Robot_ObstacleRun,
    Robot_ObstacleCollision,
    Robot_TargetWait,
    Robot_TargetRun,
    Robot_TrajectoryNoise,
    Robot_Reverse,
    Robot_TargetCollision,
    Robot_TopTouch
};

struct robot_Internal
{
    sim_Time begin_noise;
    sim_Time begin_reverse;
    sim_Time begin_top_touch;

    sim_Time last_noise;
    sim_Time last_reverse;
    bool initialized;
};

struct robot_Event
{
    bool is_run_sig;
    bool is_wait_sig;
    bool is_top_touch;
    bool is_bumper;
    bool target_switch_pin;
    sim_Time elapsed_time;
};

struct robot_Action
{
    // This is the speed at the rim of the wheel (in contact with the ground).
    // The associated angular velocity, assuming no friction, is given by the
    // relationship w = vr, where
    //   w := Angular speed
    //   v := Linear speed
    //   r := Wheel radius
    float left_wheel;
    float right_wheel;
    bool red_led;
    bool green_led;

    bool was_top_touched;
    bool was_bumped;
};

struct sim_Robot
{
    robot_State state;
    robot_Internal internal;
    robot_Action action;

    float x;
    float y;
    sim_World_Angle q;

    // Physical parameters
    float L;  // distance between wheels (m)
    float vl; // left-wheel speed (m/s)
    float vr; // right-wheel speed (m/s)

    float forward_x;
    float forward_y;

    bool removed;
};

struct sim_Drone
{
    float x;
    float y;
    float xr;
    float yr;
    float v_max;
    sim_Command cmd;
    bool cmd_done;

    bool landing;
    float land_timer;
};

struct sim_State
{
    // Random-number-generator state
    u32 seed;
    u32 xor128_x;
    u32 xor128_y;
    u32 xor128_z;
    u32 xor128_w;

    sim_Time elapsed_time;
    sim_Drone drone;
    sim_Robot robots[Num_Robots];
};

sim_State sim_init(u32 seed);

// Advances the world state forward Sim_Timestep seconds,
// Remarks
//     Increasing Sim_Timestep will decrease the accuracy of the simulation,
//     and may cause weird bugs like missed collisions. If you would like to
//     simulate ahead a given number of seconds, I would suggest to keep the
//     timestep low and run it in a for loop for n=Interval/Sim_Timestep
//     iterations.
// Return
//     A snapshot of the world state after simulating ahead one time step.
sim_State sim_tick(sim_State state, sim_Command cmd);

#endif // SIM_HEADER_INCLUDE

// ***********************************************************************
//                        Implementation starts here
// ***********************************************************************
#include <math.h> // for abs, cos, sin

#ifdef SIM_IMPLEMENTATION
#ifndef PI
#define PI 3.14159265359f
#endif

#ifndef TWO_PI
#define TWO_PI 6.28318530718f
#endif

// Global variables
static sim_State *INTERNAL;
static sim_Robot *ROBOTS;
static sim_Robot *OBSTACLES;
static sim_Robot *TARGETS;
static sim_Drone *DRONE;

// This algorithm has a maximal period of 2^128 − 1.
// https://en.wikipedia.org/wiki/Xorshift
static void
xor128(u32 *in_out_x,
       u32 *in_out_y,
       u32 *in_out_z,
       u32 *in_out_w)
{
    u32 x = *in_out_x;
    u32 y = *in_out_y;
    u32 z = *in_out_z;
    u32 w = *in_out_w;
    u32 t = x ^ (x << 11);
    *in_out_x = y;
    *in_out_y = z;
    *in_out_z = w;
    *in_out_w = w ^ (w >> 19) ^ t ^ (t >> 8);
}

static u32
_xor128()
{
    xor128(&INTERNAL->xor128_x,
           &INTERNAL->xor128_y,
           &INTERNAL->xor128_z,
           &INTERNAL->xor128_w);
    u32 result = INTERNAL->xor128_w;
    return result;
}

static int
random_0_64()
{
    return _xor128() % 65;
}

// ***********************************************************************
// What follows is mostly a transcription of the given IARC Mission 7
// iRobot code, available here:
//   http://www.aerialroboticscompetition.org/stories/stories4.php
// ***********************************************************************
#define Meters 1.0f
#define Millimeters 0.001f
#define Seconds 1.0f
#define Robot_Speed (330.0f * Millimeters / Seconds)

// Time between trajectory noise injections
#define Noise_Interval (5.0f * Seconds)

// Time between auto-reverse
#define Reverse_Interval (20.0f * Seconds)

// Time needed to affect trajectory
#define Noise_Length (0.850f * Seconds)

// Time needed to reverse trajectory
// (.33/2 m/s * pi * wheelbase / 2)
#define Reverse_Length (2.456f * Seconds)

// Time needed to spin 45 degrees
#define Top_Touch_Time (Reverse_Length / 4.0f)

#define TransitionTo(state)                \
    {                                      \
    state##Start(event, internal, action); \
    return Robot_##state;                  \
    }

static void
ObstacleWaitStart(robot_Event event,
                  robot_Internal *internal,
                  robot_Action *action)
{
    action->red_led = 1;
}

static void
ObstacleRunStart(robot_Event event,
                 robot_Internal *internal,
                 robot_Action *action)
{
    action->red_led = 1;
    action->green_led = 0;
    action->left_wheel = Robot_Speed - 9 * Millimeters / Seconds;
    action->right_wheel = Robot_Speed + 9 * Millimeters / Seconds;
}

static void
ObstacleCollisionStart(robot_Event event,
                       robot_Internal *internal,
                       robot_Action *action)
{
    action->left_wheel = 0.0f;
    action->right_wheel = 0.0f;
    action->red_led = 1;
    action->green_led = 1;
}

static void
TargetWaitStart(robot_Event event,
                robot_Internal *internal,
                robot_Action *action)
{
    action->green_led = 1;
}

static void
TargetRunStart(robot_Event event,
               robot_Internal *internal,
               robot_Action *action)
{
    action->left_wheel = Robot_Speed;
    action->right_wheel = Robot_Speed;
    action->green_led = 1;
    action->red_led = 0;
}

static void
TrajectoryNoiseStart(robot_Event event,
                     robot_Internal *internal,
                     robot_Action *action)
{
    int offset = random_0_64() - 32;
    float offset_mps = (float)offset * Millimeters / Seconds;
    action->left_wheel = Robot_Speed - offset_mps;
    action->right_wheel = Robot_Speed + offset_mps;
    action->red_led = 1;
    internal->begin_noise = event.elapsed_time;
}

static void
ReverseStart(robot_Event event,
             robot_Internal *internal,
             robot_Action *action)
{
    action->left_wheel = -Robot_Speed / 2.0f;
    action->right_wheel = Robot_Speed / 2.0f;
    action->red_led = 1;
    internal->begin_reverse = event.elapsed_time;
}

static void
TargetCollisionStart(robot_Event event,
                     robot_Internal *internal,
                     robot_Action *action)
{
    action->left_wheel = 0.0f;
    action->right_wheel = 0.0f;
}

static void
TopTouchStart(robot_Event event,
              robot_Internal *internal,
              robot_Action *action)
{
    action->left_wheel = -Robot_Speed / 2.0f;
    action->right_wheel = Robot_Speed / 2.0f;
    action->red_led = 1;
    internal->begin_top_touch = event.elapsed_time;
}

static robot_State
robot_fsm(robot_State state,
          robot_Internal *internal,
          robot_Event event,
          robot_Action *action)
{
    action->was_bumped = 0;
    action->was_top_touched = 0;
    if (!internal->initialized)
    {
        internal->begin_noise = event.elapsed_time;
        internal->begin_reverse = event.elapsed_time;
        internal->begin_top_touch = event.elapsed_time;
        internal->last_noise = event.elapsed_time;
        internal->last_reverse = event.elapsed_time;

        internal->initialized = true;
    }
    switch (state)
    {
        case Robot_Start:
        {
            if (event.target_switch_pin)
            {
                TransitionTo(TargetWait);
            }
            else
            {
                TransitionTo(ObstacleWait);
            }
        } break;

        case Robot_ObstacleWait:
        {
            if (event.is_run_sig)
            {
                TransitionTo(ObstacleRun);
            }
        } break;

        case Robot_ObstacleRun:
        {
            if (event.is_wait_sig)
            {
                action->left_wheel = 0.0f;
                action->right_wheel = 0.0f;
                TransitionTo(ObstacleWait);
            }
            else if (event.is_bumper)
            {
                action->left_wheel = 0.0f;
                action->right_wheel = 0.0f;
                TransitionTo(ObstacleCollision);
            }
        } break;

        case Robot_ObstacleCollision:
        {
            if (event.is_wait_sig)
            {
                TransitionTo(ObstacleWait);
            }
            else if (!event.is_bumper)
            {
                TransitionTo(ObstacleRun);
            }
        } break;

        case Robot_TargetWait:
        {
            if (event.is_run_sig)
            {
                // Reset noise and reverse timers
                internal->last_noise = event.elapsed_time;
                internal->last_reverse = event.elapsed_time;
                TransitionTo(TargetRun);
            }
        } break;

        case Robot_TargetRun:
        {
            if (event.is_wait_sig)
            {
                TransitionTo(TargetWait);
            }
            else if (event.is_top_touch)
            {
                TransitionTo(TopTouch);
            }
            else if (event.elapsed_time -
                     internal->last_reverse >
                     Reverse_Interval)
            {
                internal->last_reverse = event.elapsed_time;
                TransitionTo(Reverse);
            }
            else if (event.elapsed_time - internal->last_noise > Noise_Interval)
            {
                TransitionTo(TrajectoryNoise);
            }
            else if (event.is_bumper)
            {
                TransitionTo(TargetCollision);
            }
        } break;

        case Robot_TrajectoryNoise:
        {
            if (event.is_wait_sig)
            {
                TransitionTo(TargetWait);
            }
            else if (event.is_top_touch)
            {
                TransitionTo(TopTouch);
            }
            else if (event.elapsed_time - internal->begin_noise > Noise_Length)
            {
                internal->last_noise = event.elapsed_time;
                TransitionTo(TargetRun);
            }
            else if (event.is_bumper)
            {
                TransitionTo(TargetCollision);
            }
        } break;

        case Robot_Reverse:
        {
            if (event.is_wait_sig)
            {
                TransitionTo(TargetWait);
            }
            else if (event.is_top_touch)
            {
                TransitionTo(TopTouch);
            }
            else if (event.elapsed_time - internal->begin_reverse > Reverse_Length)
            {
                TransitionTo(TargetRun);
            }
        } break;

        case Robot_TargetCollision:
        {
            action->was_bumped = 1;
            TransitionTo(Reverse);
        } break;

        case Robot_TopTouch:
        {
            action->was_top_touched = 1;
            if (event.is_wait_sig)
            {
                TransitionTo(TargetWait);
            }
            else if (event.elapsed_time - internal->begin_top_touch > Top_Touch_Time)
            {
                TransitionTo(TargetRun);
            }
            else if (event.is_bumper)
            {
                TransitionTo(TargetCollision);
            }
        } break;
    }

    // Remain in current state
    return state;
}

static sim_World_Angle
wrap_angle(sim_World_Angle angle)
{
    while (angle > TWO_PI)
        angle -= TWO_PI;
    while (angle < 0.0f)
        angle += TWO_PI;
    return angle;
}

static void
robot_integrate(sim_Robot *robot, float dt)
{
    float v = 0.5f * (robot->vl + robot->vr);
    float w = (robot->vr - robot->vl) / (robot->L*0.5f);
    robot->x += v * cos(robot->q) * dt;
    robot->y += v * sin(robot->q) * dt;
    robot->q += w * dt;

    // Wrap angles
    robot->q = wrap_angle(robot->q);

    robot->forward_x = cos(robot->q);
    robot->forward_y = sin(robot->q);
}

static float
vector_length(float dx, float dy)
{
    return sqrt(dx*dx + dy*dy);
}

sim_State sim_init(u32 seed)
{
    sim_State result;
    INTERNAL = &result;
    DRONE = &INTERNAL->drone;
    ROBOTS = INTERNAL->robots;
    TARGETS = INTERNAL->robots;
    OBSTACLES = INTERNAL->robots + Num_Targets;

    INTERNAL->elapsed_time = 0.0f;

    // Use the seed to set the initial state of the xorshift
    {
        // Pick out some pieces of the number
        // TODO: Does this pattern generate bad sequences?
        INTERNAL->seed = seed;
        INTERNAL->xor128_x = seed & 0xaa121212;
        INTERNAL->xor128_y = seed & 0x21aa2121;
        INTERNAL->xor128_z = seed & 0x1212aa12;
        INTERNAL->xor128_w = seed & 0x212a12aa;

        // Seeds must be nonzero
        if (INTERNAL->xor128_x == 0) INTERNAL->xor128_x++;
        if (INTERNAL->xor128_y == 0) INTERNAL->xor128_y++;
        if (INTERNAL->xor128_z == 0) INTERNAL->xor128_z++;
        if (INTERNAL->xor128_w == 0) INTERNAL->xor128_w++;
    }

    DRONE->x = 10.0f;
    DRONE->y = 10.0f;
    DRONE->xr = 10.0f;
    DRONE->yr = 10.0f;
    DRONE->v_max = 1.0f;
    DRONE->cmd.type = sim_CommandType_NoCommand;
    DRONE->cmd.x = 0.0f;
    DRONE->cmd.y = 0.0f;
    DRONE->cmd.i = 0;
    DRONE->landing = false;
    DRONE->cmd_done = false;
    DRONE->land_timer = 0.0f;

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

        robot.L = Sim_Robot_Wheel_Distance;

        // Spawn each ground robot in a circle
        float t = TWO_PI * i / (float)(Num_Targets);
        robot.x = 10.0f + Sim_Target_Init_Radius * cos(t);
        robot.y = 10.0f + Sim_Target_Init_Radius * sin(t);
        robot.q = t;
        robot.internal.initialized = false;
        robot.state = Robot_Start;
        robot.removed = false;

        TARGETS[i] = robot;
    }

    for (u32 i = 0; i < Num_Obstacles; i++)
    {
        float t = TWO_PI * i / (float)(Num_Obstacles);

        sim_Robot robot = {};

        robot.L = Sim_Robot_Wheel_Distance;

        // The obstacles are also spawned in a circle,
        // but at an initial radius of 5 meters.
        robot.x = 10.0f + Sim_Obstacle_Init_Radius * cos(t);
        robot.y = 10.0f + Sim_Obstacle_Init_Radius * sin(t);
        robot.q = t + PI / 2.0f;
        robot.internal.initialized = false;
        robot.state = Robot_Start;
        robot.removed = false;

        OBSTACLES[i] = robot;
    }

    INTERNAL = 0;
    DRONE = 0;
    ROBOTS = 0;
    TARGETS = 0;
    OBSTACLES = 0;

    return result;
}

sim_State sim_tick(sim_State state, sim_Command cmd)
{
    sim_State result = state;
    INTERNAL = &result;
    DRONE = &INTERNAL->drone;
    ROBOTS = INTERNAL->robots;
    TARGETS = INTERNAL->robots;
    OBSTACLES = INTERNAL->robots + Num_Targets;

    INTERNAL->elapsed_time += Sim_Timestep;

    robot_Event events[Num_Robots] = {0};
    for (u32 i = 0; i < Num_Robots; i++)
    {
        if (ROBOTS[i].removed) continue;

        events[i].is_run_sig = 0;
        events[i].is_wait_sig = 0;
        events[i].is_top_touch = 0;
        events[i].is_bumper = 0;
        events[i].target_switch_pin = 0;
        events[i].elapsed_time = INTERNAL->elapsed_time;
    }

    if (cmd.type != sim_CommandType_NoCommand)
    {
        DRONE->cmd_done = false;
        DRONE->cmd = cmd;
    }

    switch (DRONE->cmd.type)
    {
        case sim_CommandType_NoCommand:
        {
            // Keep current command
        } break;

        case sim_CommandType_LandOnTopOf:
        {
            DRONE->xr = TARGETS[cmd.i].x;
            DRONE->yr = TARGETS[cmd.i].y;
            float dx = DRONE->xr - DRONE->x;
            float dy = DRONE->yr - DRONE->y;
            float len = sqrt(dx*dx + dy*dy);
            if (len < Sim_Drone_Target_Proximity)
            {
                if (!DRONE->landing)
                {
                    DRONE->landing = true;
                    DRONE->land_timer = Sim_LandOnTopOf_Time;
                }
            }
            else
            {
                float v = DRONE->v_max / len;
                float vx = v * dx;
                float vy = v * dy;
                DRONE->x += vx * Sim_Timestep;
                DRONE->y += vy * Sim_Timestep;
            }
            if (DRONE->landing)
            {
                DRONE->land_timer -= Sim_Timestep;
                if (DRONE->land_timer < 0.0f)
                {
                    events[DRONE->cmd.i].is_top_touch = true;
                }
            }
            if (TARGETS[DRONE->cmd.i].action.was_top_touched)
            {
                DRONE->land_timer = 0.0f;
                DRONE->landing = false;
                DRONE->cmd_done = true;
                DRONE->cmd.type = sim_CommandType_NoCommand;
            }
        } break;

        case sim_CommandType_LandInFrontOf:
        {
            DRONE->xr = TARGETS[cmd.i].x;
            DRONE->yr = TARGETS[cmd.i].y;
            float dx = DRONE->xr - DRONE->x;
            float dy = DRONE->yr - DRONE->y;
            float len = sqrt(dx*dx + dy*dy);
            if (len < Sim_Drone_Target_Proximity)
            {
                if (!DRONE->landing)
                {
                    DRONE->landing = true;
                    DRONE->land_timer = Sim_LandInFrontOf_Time;
                }
            }
            else
            {
                float v = DRONE->v_max / len;
                float vx = v * dx;
                float vy = v * dy;
                DRONE->x += vx * Sim_Timestep;
                DRONE->y += vy * Sim_Timestep;
            }
            if (!DRONE->landing)
            {
                DRONE->landing = true;
                DRONE->land_timer = Sim_LandInFrontOf_Time;
            }
            if (DRONE->landing)
            {
                DRONE->land_timer -= Sim_Timestep;
                if (DRONE->land_timer < 0.0f)
                {
                    events[DRONE->cmd.i].is_bumper = true;
                }
            }
            if (TARGETS[DRONE->cmd.i].action.was_bumped)
            {
                DRONE->landing = false;
                DRONE->cmd_done = true;
                DRONE->cmd.type = sim_CommandType_NoCommand;
            }
        } break;

        case sim_CommandType_Track:
        {
            DRONE->xr = TARGETS[cmd.i].x;
            DRONE->yr = TARGETS[cmd.i].y;
            float dx = DRONE->xr - DRONE->x;
            float dy = DRONE->yr - DRONE->y;
            float len = sqrt(dx*dx + dy*dy);
            float v = DRONE->v_max / len;
            float vx = v * dx;
            float vy = v * dy;
            DRONE->x += vx * Sim_Timestep;
            DRONE->y += vy * Sim_Timestep;
        } break;

        case sim_CommandType_Search:
        {
            DRONE->xr = cmd.x;
            DRONE->yr = cmd.y;
            float dx = DRONE->xr - DRONE->x;
            float dy = DRONE->yr - DRONE->y;
            float len = sqrt(dx*dx + dy*dy);
            if (len < Sim_Drone_Goto_Proximity)
            {
                DRONE->cmd.type = sim_CommandType_NoCommand;
                DRONE->cmd_done = true;
            }
            else
            {
                float v = DRONE->v_max / len;
                float vx = v * dx;
                float vy = v * dy;
                DRONE->x += vx * Sim_Timestep;
                DRONE->y += vy * Sim_Timestep;
            }
        } break;
    }

    struct collision_info
    {
        int hits;
        int bumper_hits;
        float resolve_delta_x;
        float resolve_delta_y;
    };

    collision_info collision[Num_Robots] = {0};

    for (u32 i = 0; i < Num_Robots; i++)
    {
        if (ROBOTS[i].removed) continue;

        collision[i].hits = 0;
        collision[i].bumper_hits = 0;
        collision[i].resolve_delta_x = 0.0f;
        collision[i].resolve_delta_y = 0.0f;

        switch (ROBOTS[i].state)
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
            if (i == n || ROBOTS[n].removed)
            {
                continue;
            }
            else
            {
                float x1 = ROBOTS[i].x;
                float y1 = ROBOTS[i].y;
                float r1 = ROBOTS[i].L * 0.5f;
                float x2 = ROBOTS[n].x;
                float y2 = ROBOTS[n].y;
                float r2 = ROBOTS[n].L * 0.5f;
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
                    bool on_bumper = (dx * ROBOTS[i].forward_x +
                                      dy * ROBOTS[i].forward_y) <= 0.0f;
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
        if (ROBOTS[i].removed)
            continue;
        robot_integrate(&ROBOTS[i], Sim_Timestep);
        if (ROBOTS[i].x < -Sim_Target_Removal_Margin ||
            ROBOTS[i].x > 20.0f + Sim_Target_Removal_Margin ||
            ROBOTS[i].y < -Sim_Target_Removal_Margin ||
            ROBOTS[i].y > 20.0f + Sim_Target_Removal_Margin)
        {
            ROBOTS[i].removed = true;
        }
        if (collision[i].hits > 0)
        {
            ROBOTS[i].x += collision[i].resolve_delta_x * 1.02f;
            ROBOTS[i].y += collision[i].resolve_delta_y * 1.02f;
        }
        ROBOTS[i].state = robot_fsm(ROBOTS[i].state,
                                    &ROBOTS[i].internal,
                                    events[i],
                                    &ROBOTS[i].action);
        ROBOTS[i].vl = ROBOTS[i].action.left_wheel;
        ROBOTS[i].vr = ROBOTS[i].action.right_wheel;
    }

    #if 0
    sim_Observed_State result = {};
    result.elapsed_time = INTERNAL->elapsed_time;
    for (u32 i = 0; i < Num_Targets; i++)
    {
        result.target_in_view[i] = false; // TODO:
        result.target_x[i] = TARGETS[i].x;
        result.target_y[i] = TARGETS[i].y;
        result.target_q[i] = TARGETS[i].q;
        result.target_removed[i] = TARGETS[i].removed;
    }
    for (u32 i = 0; i < Num_Obstacles; i++)
    {
        result.obstacle_x[i] = OBSTACLES[i].x;
        result.obstacle_y[i] = OBSTACLES[i].y;
        result.obstacle_q[i] = OBSTACLES[i].q;
    }
    result.drone_x = INTERNAL->drone.x;
    result.drone_y = INTERNAL->drone.y;
    result.drone_cmd_done = DRONE->cmd_done;

    return result;
    #else
    INTERNAL = 0;
    DRONE = 0;
    ROBOTS = 0;
    TARGETS = 0;
    OBSTACLES = 0;
    return result;
    #endif
}

#endif // SIM_IMPLEMENTATION
