# RealTime-Sim-Camera
Real Time simulation of a pan-tilt camera using [ALLEGRO](https://github.com/liballeg/allegro5) and pthread library (C language) tracking a moving object in the space
with RATE MONOTONIC scheduling.

![demo](https://user-images.githubusercontent.com/95044968/218305771-e4237007-e814-4ccd-aea6-7a9e37ed1b91.gif)



## Concurrent Tasks
5 concurrent tasks sharing resources are managed:
- Moving object
- Camera motors
- Screen
- Camera screen
- GUI

## ALLEGRO Installation
The v4.2 of the [ALLEGRO](https://github.com/liballeg/allegro5) library is used. To install it please run:

`sudo apt‐get install liballegro4.2 liballegro4.2‐dev`

## Compilation
Cmake build system generator is used. To compile the code:

`mkdir build`

`cd build`

`cmake .. && cmake --build .`

## Execution
***Use administrator priviliges*** due to the use of RATE MONOTONIC scheduler:

`sudo ./camera`
