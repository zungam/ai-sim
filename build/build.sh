#!/bin/bash

if [ "$(uname -s)" == "Darwin" ]; #If running OSX
then
	g++ platform.cpp -o bin/simulator -L/Library/Frameworks -framework OpenGL `sdl2-config --cflags --libs`
else
	g++ platform.cpp -o bin/simulator -lGL `sdl2-config --cflags --libs`
fi
