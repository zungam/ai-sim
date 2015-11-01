#include "platform.h"
#include "simulator.h"
#include "SDL_opengl.h"
#include <stdio.h>
#include <math.h>

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

void
sim_load(VideoMode *mode)
{
    mode->width = 300;
    mode->height = 300;
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
}

void
sim_tick(VideoMode mode, float t, float dt)
{
    // Send a test package once per second
    persist r32 udp_send_timer = 1.0f;
    udp_send_timer -= dt;
    if (udp_send_timer <= 0.0f)
    {
        udp_send_timer = 1.0f;
        SimulationState test_state = {};
        test_state.elapsed_sim_time = t;
        test_state.drone.x = 1.0f;
        test_state.drone.y = 2.0f;
        test_state.drone.z = 3.0f;

        for (u32 i = 0; i < Num_Targets; i++)
        {
            test_state.targets[i].x = (r32)i;
            test_state.targets[i].y = (r32)(i+2);
            test_state.targets[i].q = 0.1f;
            test_state.targets[i].vl = 0.2f;
            test_state.targets[i].vr = 0.3f;
        }

        for (u32 i = 0; i < Num_Obstacles; i++)
        {
            test_state.obstacles[i].x = (r32)i * 4.0f;
            test_state.obstacles[i].y = 10.0f;
            test_state.obstacles[i].q = 0.3f;
            test_state.obstacles[i].vl = 0.2f;
            test_state.obstacles[i].vr = 0.3f;
        }

        sim_send_state(&test_state);
    }

    set_scale(mode.width/(r32)mode.height, 1.0f);
    glViewport(0, 0, mode.width, mode.height);

    r32 r = 0.2f * sin(0.3f * t + 0.11f) + 0.6f;
    r32 g = 0.1f * sin(0.4f * t + 0.55f) + 0.3f;
    r32 b = 0.3f * sin(0.5f * t + 1.44f) + 0.3f;
    glClearColor(r, g, b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glBegin(GL_LINES);
    {
        set_color(1.0f, 0.5f, 0.3f, 1.0f);
        for (u32 i = 0; i < 32; i++)
            draw_circle(0.2f*cos(1.0f*t + i / 15.0f),
                        -0.2f+0.3f*sin(1.2f*t + i / 15.0f),
                        0.3f + i / 300.0f);

        u32 wn = 4;
        for (u32 wave = 0; wave < wn; wave++)
        {
            r32 p = wave / (r32)(wn);
            u32 ln = 64;
            for (u32 i = 0; i < ln; i++)
            {
                r32 a = i / (r32)(ln-1);
                r32 b = (i+1) / (r32)(ln-1);
                r32 x0 = -1.0f + 2.0f*a;
                r32 x1 = -1.0f + 2.0f*b;
                r32 y = +0.75f;
                r32 y0 = y + (0.03f+0.03f*p)*sin(0.3f*t + (a+4.0f*p)*1.2f*PI)+0.02f*cos((2.0f+p)*t+a);
                r32 y1 = y + (0.03f+0.03f*p)*sin(0.3f*t + (b+4.0f*p)*1.2f*PI)+0.02f*cos((2.0f+p)*t+b);
                draw_line(x0, y0, x1, y1);
            }
        }
    }
    glEnd();
}
