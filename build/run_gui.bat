@echo off
set CF=-Zi -nologo -Oi -Od -WX -W4 -wd4100 -wd4189 -fp:fast /MD
set LF=-subsystem:console -incremental:no -debug SDL2.lib SDL2main.lib opengl32.lib

cl %CF% -I../lib/sdl/include ../gui.cpp /link %LF% -out:gui.exe

start gui.exe
