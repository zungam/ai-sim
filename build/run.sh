#!/bin/bash

g++ ../platform.cpp -o simulator -lGL `sdl2-config --cflags --libs`
./simulator
