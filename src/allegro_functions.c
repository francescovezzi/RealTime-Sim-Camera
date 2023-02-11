#include "allegro_functions.h"
#include <allegro.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// Returns a random float in [min, max].
float frand(float min, float max)
{
float r;
	
	r = rand()/(float)RAND_MAX;			// r [0, 1]
	return min + (max - min) * r;
}

// Returns the absolute value of a float number.
float absolute(float a)
{
	if (a >= 0) return a;
	else return -a;
}

// Returns the absolute angular distance between 2 angles who differs of |a|.
float absang(float a)
{
	if (absolute(a) <= 180) return absolute(a);
	else return 360 - absolute(a);
}

// Returns the key pressed or 0 if nothing is pressed.
char get_scancode()
{
	if (keypressed())
		return readkey() >> 8;
	else return 0;
}

// Return angular value between 0 e 359.
float ang_conv(float ang)
{
	while (ang >= 360)	{
		ang = ang - 360; 
	}
	while (ang < 0)	{
		ang = ang + 360;
	}
	return ang;
}

// Returns n if it's [a,b], a if n < a, else b.
float interval(int a, int b, float n)
{
	if (n < a) return a;
	if (n > b) return b;
	return n;
}

// Atan2f taking into account non singularities.
float smart_atan2(float a, float b)
{
	if (b >= 0 || b < 0.00001) b = b + 0.001;
	if (b < 0 || b > -0.00001) b = b - 0.001;
	return ang_conv((180 / 3.14159) * atan2f(a, b));
}

float distanza(int x1, int y1, int x2, int y2)
{
	return sqrtf((x1 - x2) * (x1 - x2) + 
			(y1 - y2) * (y1 - y2));
}

