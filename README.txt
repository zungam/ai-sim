Preliminaries
-------------
Get SDL2

Linux
    $ sudo apt-get install libsdl2-dev

Windows
    On Windows prebuilt libraries are provided
    in the build directory.

Building and running
--------------------
Linux
    $ cd build
    $ gcc platform.cpp -o app -lGL `sdl2-config --cflags --libs`

Windows
    Call the build.bat file or compile manually:
    > cd build
    > cl -MD -I../lib/sdl/include ../platform.cpp /link SDL2.lib SDL2main.lib opengl32.lib -subsystem:console -out:simulator.exe
