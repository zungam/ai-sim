#define SIM_IMPLEMENTATION
#include "sim.h"

#define global static
#define persist static

#include "SDL.h"
#include "SDL_opengl.h"
#include "SDL_assert.h"
#include <stdlib.h>

#define Assert SDL_assert
#define Printf SDL_Log

struct VideoMode
{
    int width;
    int height;
    int gl_major;
    int gl_minor;
    int double_buffer;
    int depth_bits;
    int stencil_bits;
    int multisamples;

    // 0 for immediate updates, 1 for updates synchronized with the
    // vertical retrace. If the system supports it, you may
    // specify -1 to allow late swaps to happen immediately
    // instead of waiting for the next retrace.
    int swap_interval;

    // Instead of using vsync, you can specify a desired framerate
    // that the application will attempt to keep. If a frame rendered
    // too fast, it will sleep the remaining time. Leave swap_interval
    // at 0 when using this.
    int fps_lock;
};

VideoMode default_video_mode()
{
    VideoMode mode = {};
    mode.width = 500;
    mode.height = 500;
    mode.gl_major = 1;
    mode.gl_minor = 5;
    mode.double_buffer = 1;
    mode.depth_bits = 24;
    mode.stencil_bits = 8;
    mode.multisamples = 0;
    mode.swap_interval = 1;
    return mode;
}

const char *gl_error_message(GLenum error)
{
    switch (error)
    {
    case 0: return "NO_ERROR";
    case 0x0500: return "INVALID_ENUM";
    case 0x0501: return "INVALID_VALUE";
    case 0x0502: return "INVALID_OPERATION";
    case 0x0503: return "STACK_OVERFLOW";
    case 0x0504: return "STACK_UNDERFLOW";
    case 0x0505: return "OUT_OF_MEMORY";
    case 0x0506: return "INVALID_FRAMEBUFFER_OPERATION";
    default: return "UNKNOWN";
    }
}

u64 get_tick()
{
    return SDL_GetPerformanceCounter();
}

r32 get_elapsed_time(u64 begin, u64 end)
{
    u64 frequency = SDL_GetPerformanceFrequency();
    return (r32)(end - begin) / (r32)frequency;
}

r32 time_since(u64 then)
{
    u64 now = get_tick();
    return get_elapsed_time(then, now);
}

global r32 ndc_scale_x;
global r32 ndc_scale_y;

void world_to_ndc(r32 x_world, r32 y_world,
                  r32 *x_ndc, r32 *y_ndc)
{
    *x_ndc = (x_world - 10.0f) * ndc_scale_x;
    *y_ndc = (y_world - 10.0f) * ndc_scale_y;
}

void vertex2f(r32 x, r32 y)
{
    r32 x_ndc, y_ndc;
    world_to_ndc(x, y, &x_ndc, &y_ndc);
    glVertex2f(x_ndc, y_ndc);
}

void draw_line(r32 x1, r32 y1, r32 x2, r32 y2)
{
    vertex2f(x1, y1);
    vertex2f(x2, y2);
}

void fill_square(r32 x1, r32 y1, r32 x2, r32 y2)
{
    r32 x1n, y1n, x2n, y2n;
    world_to_ndc(x1, y1, &x1n, &y1n);
    world_to_ndc(x2, y2, &x2n, &y2n);
    glVertex2f(x1n, y1n);
    glVertex2f(x2n, y1n);
    glVertex2f(x2n, y2n);
    glVertex2f(x2n, y2n);
    glVertex2f(x1n, y2n);
    glVertex2f(x1n, y1n);
}

void fill_circle(r32 x, r32 y, r32 r)
{
    r32 xn, yn;
    world_to_ndc(x, y, &xn, &yn);
    u32 n = 32;
    for (u32 i = 0; i < n; i++)
    {
        r32 t1 = TWO_PI * i / (r32)n;
        r32 t2 = TWO_PI * (i + 1) / (r32)n;
        r32 x1 = x + r*cos(t1);
        r32 y1 = y + r*sin(t1);
        r32 x2 = x + r*cos(t2);
        r32 y2 = y + r*sin(t2);
        r32 x1n, y1n, x2n, y2n;
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

void draw_circle(r32 x, r32 y, r32 r, u32 n = 32)
{
    for (u32 i = 0; i < n; i++)
    {
        r32 t1 = TWO_PI * i / (r32)n;
        r32 t2 = TWO_PI * (i + 1) / (r32)n;
        r32 x1 = x + r*cos(t1);
        r32 y1 = y + r*sin(t1);
        r32 x2 = x + r*cos(t2);
        r32 y2 = y + r*sin(t2);
        draw_line(x1, y1, x2, y2);
    }
}

void draw_robot(sim_Robot robot)
{
    r32 x = robot.x;
    r32 y = robot.y;
    r32 l = robot.L;
    r32 q = robot.q;
    draw_circle(x, y, 0.5f*l);
    draw_line(x, y, x + l*cos(q), y + l*sin(q));
}

void gui_tick(VideoMode mode, r32 gui_time, r32 gui_dt)
{
    ndc_scale_x = (mode.height / (r32)mode.width) / 12.0f;
    ndc_scale_y = 1.0f / 12.0f;
    sim_Command cmd;
    cmd.type = sim_CommandType_LandInFrontOf;
    cmd.i = 0;
    sim_State state = sim_tick(cmd);

    glViewport(0, 0, mode.width, mode.height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glLineWidth(2.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBegin(GL_LINES);
    {
        // draw grid
        glColor4f(0.87f, 0.93f, 0.84f, 0.2f);
        for (u32 i = 0; i <= 20; i++)
        {
            float x = (float)i;
            draw_line(x, 0.0f, x, 20.0f);
            draw_line(0.0f, x, 20.0f, x);
        }

        // draw visible region
        glColor4f(0.87f, 0.93f, 0.84f, 0.5f);
        draw_circle(DRONE->x, DRONE->y, 2.5f);

        // draw green line
        glColor4f(0.43f, 0.67f, 0.17f, 1.0f);
        draw_line(0.0f, 20.0f, 20.0f, 20.0f);

        // draw targets
        glColor4f(0.85, 0.83, 0.37, 1.0f);
        for (u32 i = 0; i < Num_Targets; i++)
            draw_robot(TARGETS[i]);

        // draw obstacles
        glColor4f(0.43, 0.76, 0.79, 1.0f);
        for (u32 i = 0; i < Num_Obstacles; i++)
            draw_robot(OBSTACLES[i]);

        // draw drone
        glColor4f(0.87f, 0.93f, 0.84f, 0.5f);
        draw_line(DRONE->x - 0.5f, DRONE->y,
                  DRONE->x + 0.5f, DRONE->y);
        draw_line(DRONE->x, DRONE->y - 0.5f,
                  DRONE->x, DRONE->y + 0.5f);

        // draw drone goto
        glColor4f(0.87f, 0.93f, 0.84f, 0.5f);
        draw_circle(DRONE->xr, DRONE->yr, 0.45f);

        // draw indicators of magnet or bumper activations
        for (u32 i = 0; i < Num_Targets; i++)
        {
            float x = TARGETS[i].x;
            float y = TARGETS[i].y;
            if (TARGETS[i].action.was_bumped)
            {
                glColor4f(1.0f, 0.3f, 0.1f, 1.0f);
                draw_circle(x, y, 0.5f);
            }
            else if (TARGETS[i].action.was_top_touched)
            {
                glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
                draw_circle(x, y, 0.5f);
            }
        }
    }
    glEnd();
}

int main(int argc, char *argv[])
{
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
    {
        Printf("Failed to initialize SDL: %s\n", SDL_GetError());
        return -1;
    }

    VideoMode mode = default_video_mode();

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, mode.gl_major);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, mode.gl_minor);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER,          mode.double_buffer);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,            mode.depth_bits);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE,          mode.stencil_bits);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS,    mode.multisamples > 0 ? 1 : 0);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES,    mode.multisamples);

    SDL_Window *window = SDL_CreateWindow(
        "World Simulator",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        mode.width, mode.height,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

    if (!window)
    {
        Printf("Failed to create a window: %s\n", SDL_GetError());
        return -1;
    }

    SDL_GLContext context = SDL_GL_CreateContext(window);

    // Note: This must be set on a valid context
    SDL_GL_SetSwapInterval(mode.swap_interval);

    SDL_GL_GetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, &mode.gl_major);
    SDL_GL_GetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, &mode.gl_minor);
    SDL_GL_GetAttribute(SDL_GL_DOUBLEBUFFER,          &mode.double_buffer);
    SDL_GL_GetAttribute(SDL_GL_DEPTH_SIZE,            &mode.depth_bits);
    SDL_GL_GetAttribute(SDL_GL_STENCIL_SIZE,          &mode.stencil_bits);
    SDL_GL_GetAttribute(SDL_GL_MULTISAMPLESAMPLES,    &mode.multisamples);
    mode.swap_interval = SDL_GL_GetSwapInterval();

    #if 0
    gui_init_msgs(true);
    #endif

    sim_init((int)get_tick(), 0);

    bool running = true;
    u64 initial_tick = get_tick();
    u64 last_frame_t = initial_tick;
    r32 elapsed_time = 0.0f;
    r32 delta_time = 1.0f / 60.0f;
    while (running)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
                case SDL_WINDOWEVENT:
                {
                    switch (event.window.event)
                    {
                        case SDL_WINDOWEVENT_SIZE_CHANGED:
                        {
                            Printf("Window %d size changed to %dx%d\n",
                                    event.window.windowID,
                                    event.window.data1,
                                    event.window.data2);
                            mode.width = event.window.data1;
                            mode.height = event.window.data2;
                        } break;
                    }
                } break;

                case SDL_KEYDOWN:
                {
                    if (event.key.keysym.sym == SDLK_ESCAPE)
                        running = false;
                    if (event.key.keysym.sym == SDLK_r)
                    {
                        sim_init((int)get_tick(), 0);
                    }
                } break;

                case SDL_QUIT:
                {
                    running = false;
                } break;
            }
        }
        gui_tick(mode, elapsed_time, delta_time);
        SDL_GL_SwapWindow(window);

        delta_time = time_since(last_frame_t);
        if (mode.fps_lock > 0)
        {
            r32 target_time = 1.0f / (r32)mode.fps_lock;
            r32 sleep_time = target_time - delta_time;
            if (sleep_time >= 0.01f)
                SDL_Delay((u32)(sleep_time * 1000.0f));
            delta_time = time_since(last_frame_t);
        }
        last_frame_t = get_tick();
        elapsed_time = time_since(initial_tick);

        GLenum error = glGetError();
        if (error != GL_NO_ERROR)
        {
            Printf("An error occurred: %s\n", gl_error_message(error));
            running = false;
        }
    }

    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
