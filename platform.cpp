#include "SDL.h"
#include "SDL_opengl.h"
#include "SDL_assert.h"
#include <stdlib.h>

#define Assert SDL_assert
#define Printf SDL_Log

#ifndef WINDOW_WIDTH
#define WINDOW_WIDTH 640
#endif

#ifndef WINDOW_HEIGHT
#define WINDOW_HEIGHT 480
#endif

#include "platform.h"
#include "simulator.cpp"

VideoMode default_video_mode()
{
    VideoMode mode = {};
    mode.width = 640;
    mode.height = 480;
    mode.gl_major = 3;
    mode.gl_minor = 1;
    mode.double_buffer = 1;
    mode.depth_bits = 8;
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

float get_elapsed_time(u64 begin, u64 end)
{
    u64 frequency = SDL_GetPerformanceFrequency();
    return (float)(end - begin) / (float)frequency;
}

float time_since(u64 then)
{
    u64 now = get_tick();
    return get_elapsed_time(then, now);
}

float frand()
{
    return (float)rand() / (float)RAND_MAX;
}

int random_0_64()
{
    // It's bad but whatever
    // https://channel9.msdn.com/Events/GoingNative/2013/rand-Considered-Harmful
    return rand() % 65;
}

int main(int argc, char *argv[])
{
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
    {
        Printf("Failed to initialize SDL: %s\n", SDL_GetError());
        return -1;
    }

    srand((int)get_tick());

    VideoMode mode = default_video_mode();
    sim_load(&mode);

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

    sim_init(mode);

    bool running = true;
    u64 initial_tick = get_tick();
    u64 last_frame_t = initial_tick;
    float elapsed_time = 0.0f;
    float delta_time = 1.0f / 60.0f;
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
                } break;

                case SDL_QUIT:
                {
                    running = false;
                } break;
            }
        }
        sim_tick(mode, elapsed_time, delta_time);
        SDL_GL_SwapWindow(window);

        delta_time = time_since(last_frame_t);

        // TODO: FPS lock
        #if 0
        float sleep_time = FRAME_TIME - delta_time;
        if (sleep_time >= 0.0f && sleep_time >= SLEEP_GRANULARITY)
            SDL_Delay((u32)(sleep_time * 1000.0f));
        delta_time = time_since(last_frame_t);
        #endif
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
