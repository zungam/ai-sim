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

typedef float time_stamp;

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
    time_stamp begin_noise;
    time_stamp begin_reverse;
    time_stamp begin_top_touch;

    time_stamp last_noise;
    time_stamp last_reverse;
    bool initialized;
};

struct robot_Event
{
    bool is_run_sig;
    bool is_wait_sig;
    bool is_top_touch;
    bool is_bumper;
    bool target_switch_pin;
    time_stamp elapsed_sim_time;
};

typedef float speed;
struct robot_Action
{
    // This is the speed at the rim of the wheel (in contact with the ground).
    // The associated angular velocity, assuming no friction, is given by the
    // relationship w = vr, where
    //   w := Angular speed
    //   v := Linear speed
    //   r := Wheel radius
    speed left_wheel;
    speed right_wheel;
    bool red_led;
    bool green_led;

    bool was_top_touched;
    bool was_bumped;

    // I don't include passive/safe mode flags as
    // in the original iRobot IARC ground robot code.
    // They seem fairly useless for simulation.
    // See http://www.irobot.com/filelibrary/pdfs/hrd/create/Create%20Open%20Interface_v2.pdf
};

#define TransitionTo(state)                \
    {                                      \
    state##Start(event, internal, action); \
    return Robot_##state;                  \
    }

void ObstacleWaitStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    action->red_led = 1;
}

void ObstacleRunStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    action->red_led = 1;
    action->green_led = 0;
    action->left_wheel = Robot_Speed - 9 * Millimeters / Seconds;
    action->right_wheel = Robot_Speed + 9 * Millimeters / Seconds;
}

void ObstacleCollisionStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    action->left_wheel = 0.0f;
    action->right_wheel = 0.0f;
    action->red_led = 1;
    action->green_led = 1;
}

void TargetWaitStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    action->green_led = 1;
}

void TargetRunStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    action->left_wheel = Robot_Speed;
    action->right_wheel = Robot_Speed;
    action->green_led = 1;
    action->red_led = 0;
}

void TrajectoryNoiseStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    int offset = random_0_64() - 32;
    speed offset_mps = (speed)offset * Millimeters / Seconds;
    action->left_wheel = Robot_Speed - offset_mps;
    action->right_wheel = Robot_Speed + offset_mps;
    action->red_led = 1;
    internal->begin_noise = event.elapsed_sim_time;
}

void ReverseStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    action->left_wheel = +Robot_Speed / 2.0f;
    action->right_wheel = -Robot_Speed / 2.0f;
    action->red_led = 1;
    internal->begin_reverse = event.elapsed_sim_time;
}

void TargetCollisionStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    action->left_wheel = 0.0f;
    action->right_wheel = 0.0f;
}

void TopTouchStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    action->left_wheel = +Robot_Speed / 2.0f;
    action->right_wheel = -Robot_Speed / 2.0f;
    action->red_led = 1;
    internal->begin_top_touch = event.elapsed_sim_time;
}

robot_State robot_fsm(robot_State state,
                      robot_Internal *internal,
                      robot_Event event,
                      robot_Action *action)
{
    action->was_bumped = 0;
    action->was_top_touched = 0;
    if (!internal->initialized)
    {
        internal->begin_noise = event.elapsed_sim_time;
        internal->begin_reverse = event.elapsed_sim_time;
        internal->begin_top_touch = event.elapsed_sim_time;
        internal->last_noise = event.elapsed_sim_time;
        internal->last_reverse = event.elapsed_sim_time;

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
                internal->last_noise = event.elapsed_sim_time;
                internal->last_reverse = event.elapsed_sim_time;
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
            else if (event.elapsed_sim_time - internal->last_reverse > Reverse_Interval)
            {
                internal->last_reverse = event.elapsed_sim_time;
                TransitionTo(Reverse);
            }
            else if (event.elapsed_sim_time - internal->last_noise > Noise_Interval)
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
            else if (event.elapsed_sim_time - internal->begin_noise > Noise_Length)
            {
                internal->last_noise = event.elapsed_sim_time;
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
            else if (event.elapsed_sim_time - internal->begin_reverse > Reverse_Length)
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
            else if (event.elapsed_sim_time - internal->begin_top_touch > Top_Touch_Time)
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
