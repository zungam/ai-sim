#include <stdio.h>
#include "platform.h"
#include "SDL_opengl.h"
#include <math.h>
void sim_load(VideoMode *mode)
{
    mode->width = 300;
    mode->height = 300;
    mode->gl_major = 3;
    mode->gl_minor = 1;
    mode->double_buffer = 1;
    mode->depth_bits = 24;
    mode->stencil_bits = 8;
    mode->multisamples = 0;
    mode->swap_interval = 1;
    printf("--load\n");
}

void sim_init(VideoMode mode)
{
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
}

void sim_tick(VideoMode mode, float t, float dt)
{
    glViewport(0, 0, mode.width, mode.height);

    r32 r = 0.2f * sin(0.3f * t + 0.11f) + 0.6f;
    r32 g = 0.1f * sin(0.4f * t + 0.55f) + 0.3f;
    r32 b = 0.3f * sin(0.5f * t + 1.44f) + 0.3f;
    glClearColor(r, g, b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glBegin(GL_LINES);
    {
        u32 n = 8;
        r32 pi = 3.1415926f;
        for (u32 i = 0; i < n; i++)
        {
            r32 a = i / (r32)(n-1);
            r32 w = a + 0.3f;
            r32 p = a * pi / 8.0f;
            r32 x0 = 0.5f*cos(w*t + p)*mode.height/mode.width;
            r32 y0 = 0.5f*sin(w*t + p);
            r32 x1 = -x0;
            r32 y1 = -y0;
            glColor3f(1.0f, 1.0f, 1.0f); glVertex2f(x0, y0 + 0.3f*sin(0.3f*t));
            glColor3f(1.0f, 1.0f, 1.0f); glVertex2f(x1, y1 + 0.3f*sin(0.3f*t));
        }

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
                r32 y = -0.75f;
                r32 y0 = y + (0.03f+0.03f*p)*sin(0.3f*t + (a+4.0f*p)*1.2f*pi)+0.02f*cos((2.0f+p)*t+a);
                r32 y1 = y + (0.03f+0.03f*p)*sin(0.3f*t + (b+4.0f*p)*1.2f*pi)+0.02f*cos((2.0f+p)*t+b);
                glColor3f(1.0f, 1.0f, 1.0f); glVertex2f(x0, y0);
                glColor3f(1.0f, 1.0f, 1.0f); glVertex2f(x1, y1);
            }
        }
    }
    glEnd();
}
