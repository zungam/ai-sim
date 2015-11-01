@echo off
pushd .\build

set CF=-Zi -nologo -Oi -Od -WX -W4 -wd4100 -fp:fast /MD
set LF=-subsystem:console -incremental:no -debug SDL2.lib SDL2main.lib opengl32.lib

cl %CF% -I../lib/sdl/include ../platform.cpp /link %LF% -out:simulator.exe
simulator.exe
popd
