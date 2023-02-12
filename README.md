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
To execute the program please use ***administrator priviliges*** (due to the use of RATE MONOTONIC scheduler):

`sudo ./camera`

## Object Tracking
The camera tracks a moving target in the space and its velocity is controlled in closed-loop using target's position and velocity according to the
following scheme:

<p align="left">
<img src="https://user-images.githubusercontent.com/95044968/218309830-20abea89-98a5-45bd-bf17-b206cde6bc29.jpg" width="70%" height="70%"/>
</p>

## GUI
Several parameters can be changed by the user in Real-Time.
These parameters are reported in the graphic's menus.

It's possible to specify:
  - Movement type of the target:
    1. Random movement
    2. Sinusoid movement
         - Amplitude **A** and Period **P**
    3. Mouse controlled movement
  - Controller poles **x** and **y system poles**
  - Camera motors poles  **x** and **y motor poles**
  - Camera window's size **S**

To change such values just press the arrows <kbd>Up</kbd> or <kbd>Down</kbd> to select the desired one, then just press + or - to inrease or decrease it.

Press **ESC** to quit the simulation. Enjoy.


