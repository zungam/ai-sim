#define Random_Min (0)
#define Random_Max (64)
int Random_0_64()
{
    // TODO: The initial value of index acts as a simulation seed
    // Make it random
    static int index = 0;
    static int random_numbers[] = {
        0x39,  0x14,  0x28,  0x3c,  0x3c,  0x27,  0x01,  0x25,
        0x0f,  0x3f,  0x17,  0x07,  0x2e,  0x37,  0x3a,  0x0c,
        0x1b,  0x08,  0x1a,  0x0c,  0x26,  0x11,  0x00,  0x1b,
        0x0a,  0x0b,  0x3a,  0x3c,  0x30,  0x19,  0x16,  0x18,
        0x0e,  0x26,  0x14,  0x10,  0x12,  0x37,  0x3d,  0x38,
        0x3e,  0x0f,  0x08,  0x39,  0x29,  0x3f,  0x0f,  0x10,
        0x09,  0x26,  0x40,  0x0e,  0x01,  0x04,  0x19,  0x14,
        0x1b,  0x11,  0x14,  0x1d,  0x29,  0x40,  0x06,  0x39,
        0x37,  0x34,  0x1b,  0x26,  0x18,  0x33,  0x35,  0x3a,
        0x36,  0x2e,  0x31,  0x40,  0x3f,  0x28,  0x37,  0x07,
        0x34,  0x24,  0x31,  0x30,  0x1c,  0x39,  0x2c,  0x23,
        0x1c,  0x28,  0x22,  0x34,  0x22,  0x3c,  0x09,  0x34,
        0x10,  0x26,  0x10,  0x29,  0x29,  0x00,  0x2d,  0x26,
        0x40,  0x22,  0x40,  0x3c,  0x2c,  0x21,  0x40,  0x15,
        0x09,  0x0d,  0x17,  0x32,  0x34,  0x1a,  0x07,  0x32,
        0x01,  0x2b,  0x30,  0x0f,  0x18,  0x12,  0x10,  0x33,
        0x18,  0x05,  0x31,  0x34,  0x1c,  0x2e,  0x09,  0x35,
        0x00,  0x10,  0x10,  0x11,  0x0b,  0x10,  0x15,  0x03,
        0x09,  0x1b,  0x09,  0x0c,  0x2a,  0x16,  0x2b,  0x35,
        0x1f,  0x23,  0x15,  0x19,  0x05,  0x10,  0x0a,  0x28,
        0x33,  0x1f,  0x27,  0x19,  0x35,  0x3f,  0x12,  0x0a,
        0x3b,  0x23,  0x06,  0x1c,  0x19,  0x19,  0x05,  0x05,
        0x1e,  0x02,  0x2c,  0x24,  0x12,  0x1b,  0x2c,  0x1c,
        0x2c,  0x3e,  0x37,  0x20,  0x00,  0x10,  0x02,  0x17,
        0x32,  0x0f,  0x21,  0x38,  0x12,  0x38,  0x2c,  0x36,
        0x2c,  0x0c,  0x14,  0x2a,  0x19,  0x0b,  0x3e,  0x1b,
        0x16,  0x37,  0x15,  0x0e,  0x29,  0x24,  0x0a,  0x2a,
        0x37,  0x07,  0x0a,  0x0a,  0x3d,  0x40,  0x31,  0x0f,
        0x01,  0x3b,  0x12,  0x28,  0x01,  0x25,  0x1e,  0x2d,
        0x38,  0x38,  0x3a,  0x3b,  0x13,  0x2d,  0x07,  0x07,
        0x14,  0x28,  0x33,  0x3d,  0x24,  0x3c,  0x31,  0x31,
        0x05,  0x30,  0x40,  0x07,  0x14,  0x12,  0x30,  0x12,
        0x1c,  0x03,  0x05,  0x02,  0x0a,  0x15,  0x3b,  0x38,
        0x40,  0x2b,  0x0d,  0x3b,  0x1d,  0x00,  0x08,  0x0b,
        0x1c,  0x12,  0x24,  0x04,  0x04,  0x30,  0x14,  0x3d,
        0x20,  0x15,  0x3f,  0x20,  0x0e,  0x15,  0x1c,  0x14,
        0x28,  0x14,  0x20,  0x0c,  0x17,  0x3b,  0x16,  0x32,
        0x14,  0x3d,  0x06,  0x22,  0x36,  0x3c,  0x11,  0x14,
        0x0f,  0x29,  0x14,  0x09,  0x3a,  0x0a,  0x18,  0x1b,
        0x24,  0x30,  0x3f,  0x2e,  0x07,  0x1a,  0x16,  0x25,
        0x0b,  0x0c,  0x12,  0x10,  0x06,  0x0c,  0x16,  0x29,
        0x27,  0x0b,  0x24,  0x03,  0x18,  0x33,  0x2f,  0x03,
        0x06,  0x32,  0x19,  0x32,  0x3c,  0x0e,  0x0f,  0x0c,
        0x15,  0x2a,  0x17,  0x1a,  0x07,  0x01,  0x15,  0x1c,
        0x38,  0x3a,  0x37,  0x09,  0x24,  0x24,  0x00,  0x1b,
        0x32,  0x26,  0x14,  0x2e,  0x22,  0x35,  0x38,  0x1d,
        0x07,  0x1b,  0x02,  0x00,  0x32,  0x0f,  0x33,  0x38,
        0x36,  0x12,  0x26,  0x17,  0x0c,  0x14,  0x38,  0x3a,
        0x29,  0x03,  0x12,  0x1b,  0x24,  0x1a,  0x3c,  0x3c,
        0x10,  0x3e,  0x13,  0x3f,  0x0d,  0x0b,  0x13,  0x0a,
        0x40,  0x0a,  0x2d,  0x20,  0x18,  0x28,  0x05,  0x3a,
        0x31,  0x3b,  0x40,  0x31,  0x02,  0x0a,  0x15,  0x39,
        0x29,  0x08,  0x2e,  0x1a,  0x04,  0x05,  0x12,  0x3c,
        0x1e,  0x2f,  0x2f,  0x26,  0x11,  0x2e,  0x31,  0x1b,
        0x1c,  0x0a,  0x37,  0x00,  0x40,  0x05,  0x21,  0x2c,
        0x11,  0x26,  0x1c,  0x07,  0x3e,  0x3f,  0x06,  0x2f,
        0x36,  0x2d,  0x3d,  0x1c,  0x14,  0x36,  0x39,  0x21,
        0x3b,  0x2e,  0x20,  0x1b,  0x36,  0x2d,  0x0d,  0x1f,
        0x00,  0x32,  0x15,  0x16,  0x18,  0x21,  0x0f,  0x34,
        0x3e,  0x0f,  0x2c,  0x02,  0x0e,  0x01,  0x1e,  0x23,
        0x31,  0x09,  0x33,  0x28,  0x1f,  0x07,  0x0d,  0x3a,
        0x38,  0x14,  0x2e,  0x2e,  0x12,  0x16,  0x03,  0x38,
        0x3d,  0x39,  0x37,  0x0f,  0x15,  0x1e,  0x01,  0x00,
        0x01,  0x08,  0x17,  0x09,  0x3a,  0x05,  0x25,  0x36
    };

    int result = random_numbers[index];
    index = (index + 1) % (sizeof(random_numbers) / sizeof(int));
    return result;
}

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
    // printf("Transition to ObstacleWait\n");
    action->red_led = 1;
}

void ObstacleRunStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    // printf("Transition to ObstacleRun\n");
    action->red_led = 1;
    action->green_led = 0;
    action->left_wheel = Robot_Speed - 9 * Millimeters / Seconds;
    action->right_wheel = Robot_Speed + 9 * Millimeters / Seconds;
}

void ObstacleCollisionStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    // printf("Transition to ObstacleCollision\n");
    action->left_wheel = 0.0f;
    action->right_wheel = 0.0f;
    action->red_led = 1;
    action->green_led = 1;
}

void TargetWaitStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    // printf("Transition to TargetWait\n");
    action->green_led = 1;
}

void TargetRunStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    // printf("Transition to TargetRun\n");
    action->left_wheel = Robot_Speed;
    action->right_wheel = Robot_Speed;
    action->green_led = 1;
    action->red_led = 0;
}

void TrajectoryNoiseStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    // printf("Transition to TrajectoryNoise\n");
    int offset = Random_0_64() - Random_Max / 2;
    speed offset_mps = (speed)offset * Millimeters / Seconds;
    action->left_wheel = Robot_Speed - offset_mps;
    action->right_wheel = Robot_Speed + offset_mps;
    action->red_led = 1;
    internal->begin_noise = event.elapsed_sim_time;
}

void ReverseStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    // printf("Transition to Reverse\n");
    action->left_wheel = -Robot_Speed / 2.0f;
    action->right_wheel = Robot_Speed / 2.0f;
    action->red_led = 1;
    internal->begin_reverse = event.elapsed_sim_time;
}

void TargetCollisionStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    // printf("Transition to TargetCollision\n");
    action->left_wheel = 0.0f;
    action->right_wheel = 0.0f;
}

void TopTouchStart(robot_Event event, robot_Internal *internal, robot_Action *action)
{
    // printf("Transition to TopTouch\n");
    action->left_wheel = -Robot_Speed / 2.0f;
    action->right_wheel = Robot_Speed / 2.0f;
    action->red_led = 1;
    internal->begin_top_touch = event.elapsed_sim_time;
}

robot_State robot_fsm(robot_State state,
                      robot_Internal *internal,
                      robot_Event event,
                      robot_Action *action)
{
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
                TransitionTo(Reverse);
            }
            else if (event.elapsed_sim_time - internal->last_noise > Noise_Interval)
            {
                TransitionTo(TrajectoryNoise);
            }
            else
            {
                if (event.is_bumper)
                {
                    TransitionTo(TargetCollision);
                }
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
                internal->last_reverse = event.elapsed_sim_time;
                TransitionTo(TargetRun);
            }
        } break;

        case Robot_TargetCollision:
        {
            TransitionTo(Reverse);
        } break;

        case Robot_TopTouch:
        {
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
