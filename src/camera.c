//-----------------------------------------------------------------------------
//					  REAL TIME SYSTEMS PROJECT
//
//	CAMERA.C:	simulation of a pan-tilt camera tracking a moving object 
// 				in the space
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// TO COMPILE RUN AS ROOT:
// gcc camera.c thread_functions.o  allegro_functions.o -o camera 
// `allegro-config  --libs` -lpthread -lrt -lm
//-----------------------------------------------------------------------------

#include <stdlib.h>
#include <stdio.h>
#include <allegro.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include "allegro_functions.h"
#include "thread_functions.h"

// GLOBAL CONSTANTS

//-----------------------------------------------------------------------------
// THREAD CONSTANTS
//-----------------------------------------------------------------------------
#define STK_SIZE 4096						// 4 MB
#define TASK_PER 20							// Period of tasks [ms]
#define _GNU_SOURCE							// To use PIP

//-----------------------------------------------------------------------------
// GRAPHIC CONSTANTS
//-----------------------------------------------------------------------------
#define XWIN		800						// Window x resolution
#define YWIN		600						// Window y resoluiton
#define FSTLX		8						// Starts background first line x
#define SNDLX		564						// Starts background second line x
#define TRDLX		572						// Starts background third line x
#define FTHLX		792						// Starts background fourth line x
#define FSTLY		8 						// Starts background first line y
#define SNDLY		68						// Starts background second line y
#define TRDLY		78						// Starts background third line y
#define FTHLY		592						// Starts background fourth line y

#define X_CENTER_T	((SNDLX - FSTLX) / 2)	// X center in target box
#define Y_CENTER_T	((FTHLY - TRDLY) / 2)	// Y center in target box

#define XINCR_M		120						// X increment among words in menu
#define XINCR_S		134						// X increment in status box
#define YINCR_M		20						// Y increment in menu box
#define YINCR_S		40						// Y increment in status box
#define XBASE_M		(FSTLX +  8)			// X start of menu options
#define YBASE_M		(FSTLY + YINCR_M)		// Y start of menu options
#define K			24                      // translate menu options
#define XBASE_S		(30 + TRDLX)			// X start of status options
#define YBASE_S		(YBASE_M  + 20)			// Y start of status options

#define DIM_SEL		4						// Dim selective sphere

//-----------------------------------------------------------------------------
// TARGET CONSTANT
//-----------------------------------------------------------------------------
#define L 			10 					// Triangular object height
#define H 			25					// Half length of the base
#define TRG_SCALE 	1 					// Target size scale factor
#define PI 			(3.14159)			// PI greco value
#define CONV 		(PI / 180)			// Deg to rad

#define VMIN 		12					// Min vtarget velocity
#define VMAX 		32					// Max target velocity

#define ROT_STEP 	3					// Control rotation steps
#define ROT_TOLL	0.1					// Rotation tollerance
#define TSCALE	 	0.1					// Target time scale factor
#define DELTA_V		16					// Max velocity random variation
#define DELTA_A 	15					// Max angular variation
#define TR			3					// To handle random motion

#define AMP_MAX 	35					// Max sine amplitude
#define AMP_MIN 	5					// Min sine amplitude
#define PER_MAX 	200					// Max period sine wave
#define PER_MIN		20					// Min period sine wave
#define N			80					// #samples in a period
#define V_SIN		25					// Velocity in sinusoidal motion

#define TL 			70					// Trail length
#define MOUSE_DIM 	1 					// Mouse's click dimension
#define D_TOLL 		0.1					// Tollerance on mouse position
#define V_MOUSE		25					// Velocity in mouse movement

//-----------------------------------------------------------------------------
// USER INTERFACE CONSTANTS
//-----------------------------------------------------------------------------
#define SEL_MAX		7					// Max selectable element in status
#define SEL_MIN		0					// Min selectable element in status

//-----------------------------------------------------------------------------
// CAMERA CONSTANTS
//-----------------------------------------------------------------------------
#define SMAX		180					// Max window's size
#define SMIN		80					// Min window's size
#define S_STEP		5					// Increase or decrease window's size
#define X_CENTER_ST	(TRDLX + 110)		// X center of camera display
#define Y_CENTER_ST	(FTHLY - 140)		// Y center of camera disply
#define CAM_TOLL 	2					// Distance camera needs for moving

//-----------------------------------------------------------------------------
// MOTORS CONSTANTS
//-----------------------------------------------------------------------------
#define A_MAX		4					// The highest frequency motor pole
#define A_MIN		0.8					// Lowest frequncy motor pole
#define A_STEP		0.2					// Increase/decrease pole frequency
#define P_MAX		0.98				// Max frequency pole camera dynamic
#define P_MIN		0.70				// Max frequency pole camera dynamic
#define P_STEP		0.02				// Increase/decrease pole frequency

//-----------------------------------------------------------------------------
// GLOBAL DATA STRUCTURES
//-----------------------------------------------------------------------------
struct target	{					// Target structure
	float x;						// X origin coordinate
	float y;						// Y origin coordinate
	float alpha;					// Orientation degrees [0, 359]
	float v;						// Origin velocity module
};

struct cbuf	{						// Trak mouse trail
	int top;						// Point to the current element
	int x[TL];						// X component
	int y[TL];						// Y component
	int ragg[TL];					// Flag: point reached
};

struct click {						// Store last mouse's click position
	int x;
	int y;
	int happ;						// Flag: click happened
};

struct movimento {					// To handle target movement
	int rot;						// Flag rotation 
	int coll;						// Flag collision
	int raggiunto;					// Flag arrived
	int change;						// Flag change angle
	int tremors;					// Flag to handle random motion
	float beta;						// Rotation target
};

struct sine_wave {					// Make sinusoidal motion
	float per;						// Period
	float amp;						// Amplitude
	float fase;						// Phase
	float ls;						// Length step
	float xf;						// X component to reach
	float yf;						// Y component to reach
	int rev;						// Flag to handle bouncing
	int arr;						// Flag to handle movement
	float ang;						// Motion's inclination
	float mem;						// To calculate position's increment
	int coll;						// To handle collision
};

struct camera {						// State of camera center
	float x;						// Actual value of x camera
	float y;						// Actual value of y camera
	float vx;						// Centroid velocity x
	float vy;						// Centroid velocity y
};

struct contr_camera {				// To control the camera movement
	float x_c;						// Actual value of x centroid
	float y_c;						// Actual value of y centroid
	float x_c_old;					// Previous value of x centroid
	float y_c_old;					// Previous value of y centroid
	float a_x;						// To set x motor's pole frequency
	float a_y;						// To set x motor's pole frequency
	float rif_x;					// X reference for camera center
	float rif_y;					// Y reference for camera center
	float ut_x;						// X input quantity
	float ut_y;						// Y input quantity
	float p_x;						// System pole x dynamic
	float p_y;						// System pole y dynamic
	int not_found;					// Handle target escapes from camera
	int size;						// Dimension of camera detection window
};

struct camera_detection {			// To manage data acquired by camera
	int image[SMAX][SMAX];			// To store the image detected by camera
	int col_x[SMAX];				// To calculate x centroid
	int col_y[SMAX];				// To calculate y centroid
};

//-----------------------------------------------------------------------------
// GLOBAL VARIABLES
//-----------------------------------------------------------------------------

struct target 			bersaglio;
struct cbuf 			trail;
struct click 			mouse;
struct movimento 		mov;
struct sine_wave 		sine;
struct camera 			cam;
struct contr_camera 	contr_cam;
struct camera_detection	imm;

BITMAP *bkg = NULL;							// To contain menu and status boxes
BITMAP *schermo = NULL;						// To ontain the screen's contents

struct task_par			target_tp, graphic_tp, ui_tp,	
						camera_tp, acquisition_tp;		
						
struct sched_param 		target_pr, graphic_pr, ui_pr,
						camera_pr, acquisition_pr;
						
pthread_attr_t 			target_att, graphic_att, ui_att,
						camera_att, acquisition_att;
						
pthread_t				target_id, graphic_id, ui_id,
						camera_id, acquisition_id;

pthread_mutex_t			mux_target, mux_trail, mux_mouse,
						mux_mov, mux_sine,mux_sel, mux_cam,
						mux_size, mux_polex, mux_schermo,
						mux_poley;
						 
pthread_mutexattr_t		matt_target, matt_trail, matt_mouse,
						matt_mov, matt_sine, matt_schermo,
						matt_sel, matt_cam, matt_size,
						matt_polex, matt_poley;

sem_t sem1, sem2;							// Define 2 semaphore

int end = 0;								// Flag quit the program
int n = 3;									// Flag movement (mouse mode)
int selection = 0;							// Element selected in status
char c;										// Character pressed by user

//-----------------------------------------------------------------------------
// DECLARATION OF FUNCTIONS
//-----------------------------------------------------------------------------

void init_target();
void init_mov();
void init_trail();
void init_mouse();
void init_sine();
void reinit_sine();

void draw_target();
void rotate();
void handle_bounce();
void move_target();

void draw_trail();
void store_trail();
float calc_dist(int k);
void calc_beta(int k);

void update_amp_per_ls(float *amp, float *per, float *ls);
void calc_new_point_and_dir(float amp, float per, float ls);
int collision(float x, float y);
void sine_handle_bounce(float per);
void reach_point();


void update_local_variable_control(int *s,
			float *xc, float *yc, float *xc_old, float *x,
			float *ax, float *ay, float *yc_old, float *y);
void compute_reference(float xc, float yc,
						float xc_old, float yc_old);
void cam_handle_bounce(float rifx, float rify);
void update_cam_state(float ut_x, float ut_y, int s);
void handle_not_found(int s);
void calc_gain(float *ke_x, float *ke_y,
				float *kv_x, float *kv_y);

void movement_random();
void movement_sine();
void movement_mouse();

void show_status();
void show_dmiss();
void draw();

void select_movement();
void get_coordinates(int x, int y);
void not_pressed();
void press_left_click();
void select_change_param();
void reset();
void increment();
void decrement();
void draw_sel();

void init_image();
void update_local_variable_acq(float *xc, float *yc, int *s);
void get_put_image(float xc, float yc, int s);
void calc_centroid(float xc, float yc, int s);

void init_cam();
void init_contr_cam();

void init_allegro();
void init_background();
void fill_menu();
void fill_status();
void init_target_thread(int i);
void init_graphic_thread(int i);
void init_ui_thread(int i);
void init_threads(int i);
void init_structures();
void set_mutex(int i, 
		pthread_mutex_t *mux, 
		pthread_mutexattr_t *matt);
void init_mutex(int i); 
void init_sem();
void init();

void create_tasks();
void wait_for_task_end();

void *target_task(void *p);
void *graphic_task(void *p);
void *ui_task(void *p);
void *camera_task(void *p);
void *acquisition_task(void *p);

//-----------------------------------------------------------------------------
// TARGET FUNCTIONS
//-----------------------------------------------------------------------------

// Initialize target state, spawned inside target box.
void init_target()
{
	bersaglio.x = X_CENTER_T;		
	bersaglio.y = Y_CENTER_T;
	bersaglio.alpha = 0;
	bersaglio.v = frand(VMIN, VMAX);						
}

// Initialize cbuf structure.
void init_trail()
{
int i;
	
	trail.top = 0;
	for (i = 0; i < TL; i++)	{
		trail.x[i] = -1;
		trail.y[i] = 0;
		trail.ragg[i] = 1;
	}
}

// Initialize click structure.
void init_mouse()
{
	mouse.happ = 0;
	mouse.x = -1;
	mouse.y = -1;
}

// Initialize movement structure.
void init_mov()
{
float alpha;

	alpha = bersaglio.alpha;

	mov.rot = 1;
	mov.coll = 1;
	mov.raggiunto = 0;
	mov.change = 1;
	mov.tremors = 0;
	mov.beta = alpha;
}

// Draw target in the space.
void draw_target()
{
float p1x, p1y, p2x, p2y, p3x, p3y;
float ca, sa;
float x, y, alpha;

	pthread_mutex_lock(&mux_target);
	x = bersaglio.x;
	y = bersaglio.y;
	alpha = bersaglio.alpha;
	pthread_mutex_unlock(&mux_target);
	
	ca = cos(alpha * CONV);
	sa = sin(alpha * CONV);
		
	p1x = x - L * TRG_SCALE * sa;
	p1y = y + L * TRG_SCALE * ca;
		
	p2x = x + H * TRG_SCALE * ca;
	p2y = y + H * TRG_SCALE * sa;
	
	p3x = x + L * TRG_SCALE * sa;
	p3y = y - L * TRG_SCALE * ca;
	
	triangle(schermo,						// Draw target on schermo
		FSTLX + p1x, FTHLY - p1y,
		FSTLX + p2x, FTHLY - p2y,
		FSTLX + p3x, FTHLY - p3y,
		11);
}

//-----------------------------------------------------------------------------
// Rotate target from alpha to beta (degrees) gradually by ROT_STEP and update 
// alpha state. If the rotation ends it assignes 1 to rot flag, else 0.
//  It returns the value of rot flag.
//-----------------------------------------------------------------------------
void rotate()
{
float alpha, alpha_fin, da;

	alpha = bersaglio.alpha;

	alpha_fin = ang_conv(mov.beta);							
	mov.rot = 0;											
	da = alpha_fin - alpha;
	
	if (absang(da) < ROT_TOLL) mov.rot = 1;			// No rotation needed

	if (!mov.rot) {
		if (absang(da) > ROT_STEP)	{
			if (da > 180 || 						// Clockwise rotation				
				(da < 0 && da > -180))	{		
					alpha = alpha - ROT_STEP;		// Rotation of RSTEP
			}
			else alpha = alpha + ROT_STEP;			// Anticlockwise
		}
		else {
			alpha = alpha_fin;						// Rotation of beta
			mov.rot = 1;							// Rotation  success
		}
	bersaglio.alpha = ang_conv(alpha);				// State update
	}
}

// handle borders.
void handle_bounce()
{
int outl, outr, outt, outb;
float x, y, alpha;

	x = bersaglio.x;
	y = bersaglio.y;
	alpha = bersaglio.alpha;

	outl = (x <= H * TRG_SCALE);					
	outr = (x >= SNDLX - FSTLX - H * TRG_SCALE);
	outt = (y >= FTHLY - TRDLY - H * TRG_SCALE);
	outb = (y <= H * TRG_SCALE);
	
	if (outl)	{
		bersaglio.x = H * TRG_SCALE;
	}
	if (outr)	{
		bersaglio.x = SNDLX - FSTLX - H * TRG_SCALE;
	}
	if (outt)	{
		bersaglio.y = FTHLY - TRDLY - H * TRG_SCALE;
	}
	if (outb)	{
		bersaglio.y = H * TRG_SCALE;
	}
	
	if ((outl || outr) && mov.coll)	{
		mov.beta = ang_conv(180 - alpha);				// Target angle
		mov.rot = 0;									// Start rotation
	}
	if ((outt ||outb) && mov.coll)	{
		mov.beta = ang_conv(-alpha);
		mov.rot = 0;								
	}
	if (outt || outb ||									
		outl || outr) mov.coll = 0;
	else mov.coll = 1;									// Ready new collision
}

// Move the target accoring to its state.
void move_target()
{
	rotate();												// Update alpha

	pthread_mutex_lock(&mux_target);
	if (mov.rot)	{										// Rotation ended
		bersaglio.x = bersaglio.x + 						// Update x
			bersaglio.v * cos(bersaglio.alpha * CONV) *
				 TSCALE;					
		bersaglio.y = bersaglio.y + 						// Update y
			bersaglio.v * sin(bersaglio.alpha * CONV) *
				 TSCALE;
	}
	if (n != 2)	handle_bounce();
	pthread_mutex_unlock(&mux_target);
}

// Store trail past values.
void store_trail()
{
int	k;

	if (mouse.happ)	{
		pthread_mutex_lock(&mux_trail);
		k = trail.top;
		k = (k + 1) % TL;
		if (k == mov.raggiunto) {
			mov.raggiunto = (mov.raggiunto + 1) % TL;
			mov.change = 1;
		}
		pthread_mutex_lock(&mux_mouse);
		trail.x[k] = mouse.x;
		trail.y[k] = mouse.y;
		mouse.happ = 0;
		pthread_mutex_unlock(&mux_mouse);
		trail.ragg[k] = 0;								// Not reached
		trail.top = k;
		pthread_mutex_unlock(&mux_trail);
	}
}

// Draw trail on schermo.
void draw_trail()
{
int j, k;
int x, y;

	pthread_mutex_lock(&mux_trail);
	for (j = 0; j < TL; j++) {
		k = (trail.top - j + TL) % TL;
		x = FSTLX + trail.x[k];
		y = FTHLY - trail.y[k];
		if (trail.x[k] > -1)	{
			circlefill(schermo, x, y, MOUSE_DIM, 3);
		}
	}
	pthread_mutex_unlock(&mux_trail);
}

// lets target have a random movement.
void movement_random()
{
	if (mov.rot)	{									// Prev rot ended
		if (!mov.tremors)	{
			mov.beta = ang_conv(bersaglio.alpha + 		// Rotation target
				frand(-DELTA_A, DELTA_A));
			mov.tremors = TR;
		}
		bersaglio.v = 									// Update v	
			interval(VMIN, VMAX, bersaglio.v +
				frand(-DELTA_V, DELTA_V));	
		mov.tremors--;			
	}
	move_target();
}

// Calculate distance between target and k-th element of trail array.
float calc_dist(int k)
{
float xf, yf, x, y, d;

	pthread_mutex_lock(&mux_target);
	x = bersaglio.x;
	y = bersaglio.y;
	pthread_mutex_unlock(&mux_target);
	pthread_mutex_lock(&mux_trail);
	xf = trail.x[k];
	yf = trail.y[k];
	pthread_mutex_unlock(&mux_trail);
	d = distanza(xf, yf, x, y);
	
	return d;
}

// Change beta with the angle between target and k-th element of trail array.
void calc_beta(int k)
{
float xf, yf, x, y;

	pthread_mutex_lock(&mux_target);
	x = bersaglio.x;
	y = bersaglio.y;
	pthread_mutex_unlock(&mux_target);
	pthread_mutex_lock(&mux_trail);
	xf = trail.x[k];
	yf = trail.y[k];
	pthread_mutex_unlock(&mux_trail);
	mov.beta = smart_atan2(yf - y, xf - x);
}

// Make target to move with the mouse.
void movement_mouse()
{
float xf, yf, x, y, d_iniz, d_fin;
int k, change;
	
	change = mov.change;
	store_trail();	
	k = mov.raggiunto;
	if (!trail.ragg[k])	{								// Not reached
		d_iniz = calc_dist(k);							// Dist target point
		if (d_iniz >= D_TOLL)	{
			if (mov.change)	{
				calc_beta(k);
				change = 0;								// Keep beta
			}
			move_target();
		}
		d_fin = calc_dist(k);
		if ((d_fin >= d_iniz || d_fin <= D_TOLL) &&		// Reached
			 mov.rot)	{
			pthread_mutex_lock(&mux_trail);
			pthread_mutex_lock(&mux_target);
			bersaglio.x = trail.x[k];
			bersaglio.y = trail.y[k];
			pthread_mutex_unlock(&mux_target);
			pthread_mutex_unlock(&mux_trail);
			trail.ragg[k] = 1;	
		}
	} else	if (!trail.ragg[(k + 1) % TL])	{ 			// Next not reached
		mov.raggiunto = (mov.raggiunto + 1) % TL;
		change = 1;
		}
	mov.change = change;
}

// Init sine_wave structure.
void init_sine()
{
float x, y;

	pthread_mutex_lock(&mux_target);
	x = bersaglio.x;
	y = bersaglio.y;
	pthread_mutex_unlock(&mux_target);
	bersaglio.v = V_SIN;

	sine.per = ceil((PER_MAX + PER_MIN) / 2);	
	sine.amp = ceil((AMP_MAX + AMP_MIN) / 2);	
	sine.fase = 0;
	sine.ls = sine.per / N;
	sine.xf = x;
	sine.yf = y;
	sine.ang = ang_conv(45 * (rand() % 8));			// Multiple of 45
	sine.rev = 0;								
	sine.arr = 1;
	sine.mem = 0;								
	sine.coll = 0;
}

// Initialize another time sine struct.
void reinit_sine()
{
float x, y;

	pthread_mutex_lock(&mux_target);
	x = bersaglio.x;
	y = bersaglio.y;
	pthread_mutex_unlock(&mux_target);
	bersaglio.v = V_SIN;
	
	sine.fase = 0;
	sine.xf = x;
	sine.yf = y;
	sine.ang = ang_conv(45 * (rand() % 8));	// Multiple of 45
	sine.rev = 0;								
	sine.arr = 1;
	sine.mem = 0;								
	sine.coll = 0;
}

// Updates amplitude and period
void update_amp_per_ls(float *amp, float *per, float *ls)
{
	pthread_mutex_lock(&mux_sine);
	*amp = sine.amp;
	*per = sine.per;
	*ls = sine.ls;
	pthread_mutex_unlock(&mux_sine);
}

// Calculates new target point and its direction during sine motion.
void calc_new_point_and_dir(float amp, float per, float ls)
{
float ca, sa, xt, yt, sign, xf, yf;
float xs, ys, xr, yr, yp, ya;

	xt = sine.xf;
	yt = sine.yf;
	ca = cos(sine.ang * CONV);
	sa = sin(sine.ang * CONV);
	sign = pow(-1, sine.rev);
	xs = sign * ls;
	ya = amp * 							// Actual value
		sin((2 * PI / per) * (xs +
		sine.fase));
	yp = sine.mem;						// Previous value
	ys = ya - yp;						// Calculate the increment
	xr = ca * xs - sa * ys;
	yr = sa * xs + ca * ys;
	xf = xt + xr;
	yf = yt + yr;
	
	if (!collision(xf, yf))	{					// Target feasibility
		sine.xf = xf;
		sine.yf = yf;
		sine.mem = ya;
		sine.fase = sine.fase + xs;
		sine.arr = 0;
		sine.coll = 0;
		mov.beta = smart_atan2(yr, xr);			// Calculate target angle
	} else sine_handle_bounce(per);
}

// Calculates the collision's presence.
int collision(float x, float y)
{
int outl, outr, outt, outb;

	outl = (x <= H * TRG_SCALE);					
	outr = (x >= SNDLX - FSTLX - H * TRG_SCALE);
	outt = (y >= FTHLY - TRDLY - H * TRG_SCALE);
	outb = (y <= H * TRG_SCALE);
	
	if (outl || outr ||
		outt || outb) return 1;
	else return 0;
}

// Handle bouncing during sinusoidal motion.
void sine_handle_bounce(float per)
{
	if (!sine.coll)	{							// Not coll 2 consec times
		sine.fase = (per / 2) - sine.fase;
		sine.rev = (sine.rev + 1) % 2;
		sine.coll++;
	} else {
		sine.rev = (sine.rev + 1) % 2;
	}	
}

// Move toward the target point
void reach_point()
{
float d_iniz, d_fin;

	d_iniz = distanza(sine.xf, sine.yf, 
		bersaglio.x, bersaglio.y);
	move_target();
	d_fin = distanza(sine.xf, sine.yf, 
		bersaglio.x, bersaglio.y);
	if ((d_fin > d_iniz || d_fin <= D_TOLL) &&
		mov.rot)	{								// Reached
		pthread_mutex_lock(&mux_target);
		bersaglio.x = sine.xf;
		bersaglio.y = sine.yf;
		pthread_mutex_unlock(&mux_target);
		pthread_mutex_lock(&mux_sine);
		sine.arr = 1;
		pthread_mutex_unlock(&mux_sine);
	}
}

void movement_sine()
{
int arr;
float amp, per, ls;

	pthread_mutex_lock(&mux_sine);
	arr = sine.arr;
	pthread_mutex_unlock(&mux_sine);
	
	if (arr)	{
		update_amp_per_ls(&amp, &per, &ls);
		calc_new_point_and_dir(amp, per, ls);
	} 
	reach_point();
}

//-----------------------------------------------------------------------------
// ACQUISITION FUNCTIONS
//-----------------------------------------------------------------------------

// Initializes camera structure.
void init_cam()
{
	cam.x = 286;
	cam.y = 257;
	cam.vx = 0;
	cam.vy = 0;
}

// Resets the information acquired by the camera.
void init_image()
{
int i, j;
	
	for (i = 0; i < SMAX; i++)	{
		imm.col_x[i] = 0;
		imm.col_y[i] = 0;
		for (j = 0; j < SMAX; j++)	
			imm.image[i][j] = 0;
	}
}

// Gets the image, displayes it in status and count #pixel.
void get_put_image(float xc,float yc, int s)
{
int i, j;
float x, y;
	
	sem_wait(&sem1);
	rect(schermo,
		FSTLX + xc - s / 2, FTHLY - (yc - s / 2),
		FSTLX + xc + s / 2, FTHLY - (yc + s / 2), 14);	// Around camera center

	for (i = 0; i < s; i++) 
		for (j = 0; j < s; j++)	{
			x = xc - s / 2 + i;
			y = yc - s / 2 + j;
			imm.image[i][j] = getpixel(schermo, 
								x + FSTLX, FTHLY - y);		// Acquire image
			if (imm.image[i][j] == 11)	{
				imm.col_x[i]++;								// #pixel column i
				imm.col_y[j]++;								// #pixel row j
			}
			x = X_CENTER_ST - s / 2 + i;
			y = Y_CENTER_ST + s / 2 - j;
			putpixel(schermo, x, y, imm.image[i][j]);		// Display image
		}
	rect(schermo, X_CENTER_ST - s / 2, Y_CENTER_ST + s / 2,		// In status
			X_CENTER_ST + s / 2, Y_CENTER_ST - s / 2, 14);
	sem_post(&sem2);
}

// Calculates the centroid in the camera's image.
void calc_centroid(float xc, float yc, int s)
{
int i, x_tot = 0, y_tot = 0;
float x, y, x_num = 0, y_num = 0, x_cm, y_cm;

	contr_cam.not_found = 0;
	for (i = 0; i < s; i++)	{
		x_tot += imm.col_x[i];
		y_tot += imm.col_y[i];
		x = xc - s / 2 + i;
		y = yc - s / 2 + i;
		x_num += x * imm.col_x[i];
		y_num += y * imm.col_y[i];
	}
	if (x_tot == 0 && y_tot == 0) {
		contr_cam.not_found = 1;
		if (s < SMAX)	{
			s += 1;
			contr_cam.size = s;
		}	
	} else	{
		x_cm = x_num / x_tot;
		y_cm = y_num / y_tot;
	}
	if (contr_cam.not_found == 0)	{
		pthread_mutex_lock(&mux_cam);
		contr_cam.x_c_old = contr_cam.x_c;
		contr_cam.y_c_old = contr_cam.y_c;
		contr_cam.x_c = x_cm;
		contr_cam.y_c = y_cm;
		pthread_mutex_unlock(&mux_cam);
	}
}

// To update local variable in acquisition task.
void update_local_variable_acq(float *xc, float *yc, int *s)
{
	pthread_mutex_lock(&mux_cam);
	*xc = cam.x;
	*yc = cam.y;
	pthread_mutex_unlock(&mux_cam);
	pthread_mutex_lock(&mux_size);
	*s = contr_cam.size;
	pthread_mutex_unlock(&mux_size);
}

//-----------------------------------------------------------------------------
// CAMERA FUNCTIONS
//-----------------------------------------------------------------------------

// Initializes contr_cam structure.
void init_contr_cam()
{	
	contr_cam.x_c = 286;					
	contr_cam.y_c = 257;	
	contr_cam.x_c_old = X_CENTER_T;					
	contr_cam.y_c_old = Y_CENTER_T;				
	contr_cam.a_x = (A_MAX + A_MIN) / 2;					
	contr_cam.a_y = (A_MAX + A_MIN) / 2;				
	contr_cam.rif_x = X_CENTER_T;			
	contr_cam.rif_y = Y_CENTER_T;
	contr_cam.ut_x = 0;
	contr_cam.ut_y = 0;		
	contr_cam.p_x = (P_MAX + P_MIN) / 2;
	contr_cam.p_y = (P_MAX + P_MIN) / 2;
	contr_cam.not_found = 0;
	contr_cam.size = (SMAX + SMIN) / 2;
}

// Updates local variable.
void update_local_variable_control(int *s,
			float *xc, float *yc, float *xc_old, float *x,
			float *ax, float *ay, float *yc_old, float *y)
{
	pthread_mutex_lock(&mux_cam);
	*xc = contr_cam.x_c;
	*yc = contr_cam.y_c;
	*xc_old = contr_cam.x_c_old;
	*yc_old = contr_cam.y_c_old;
	*ax = contr_cam.a_x;
	*ay = contr_cam.a_y;
	*x = cam.x;
	*y = cam.y;
	pthread_mutex_unlock(&mux_cam);
	pthread_mutex_lock(&mux_size);
	*s = contr_cam.size;
	pthread_mutex_unlock(&mux_size);
}

// Computes the reference in position for camera.
void compute_reference(float xc, float yc,
						float xc_old, float yc_old)
{
float rifx, rify;

	rifx = 2 * xc - xc_old;	
	rify = 2 * yc - yc_old;
	cam_handle_bounce(rifx, rify);
}

// Camera reference must be inside the target box.
void cam_handle_bounce(float rifx, float rify)
{
	if (rifx < H * TRG_SCALE)
		rifx = H * TRG_SCALE;
		
	if (rifx > SNDLX - FSTLX - H * TRG_SCALE)
		rifx = SNDLX - FSTLX - H * TRG_SCALE;
		
	if (rify < H * TRG_SCALE)
		rify = H * TRG_SCALE;
		
	if (rify > FTHLY - TRDLY - H * TRG_SCALE)
		rify = FTHLY - TRDLY - H * TRG_SCALE;
		
	if (distanza(rifx, rify,
				contr_cam.rif_x, contr_cam.rif_y) >= CAM_TOLL) {
		contr_cam.rif_x = rifx;
		contr_cam.rif_y = rify;
	}
}

// Updates cam state taking into account the borders.
void update_cam_state(float ut_x, float ut_y, int s)
{
float vx, vy, x, y;

	vx = cam.vx + ut_x;
	vy = cam.vy + ut_y;
	x = cam.x + vx * TSCALE;
	y = cam.y + vy * TSCALE;
	
	if (x <= s / 2 + 1)	{
		x = s / 2 + 1;
		vx = 0;
	}
	if (x >= SNDLX - FSTLX - s / 2 - 1)	{
		x = SNDLX - FSTLX - s / 2 - 1;
		vx = 0;
	}
	if (y <= s / 2 + 1)	{
		y = s / 2 + 1;
		vy = 0;
	}
	if (y >= FTHLY - TRDLY - s / 2 - 1)	{
		y = FTHLY - TRDLY - s / 2 - 1;
		vy = 0;
	}
	pthread_mutex_lock(&mux_cam);
	cam.x = x;
	cam.y = y;
	cam.vx = vx;
	cam.vy = vy;
	pthread_mutex_unlock(&mux_cam);
	contr_cam.ut_x = ut_x;
	contr_cam.ut_y = ut_y;
}

// Manages the case the target escapes from camera
void handle_not_found(int s)
{
	if (s == SMAX)	{
		contr_cam.rif_x = X_CENTER_T;
		contr_cam.rif_y = Y_CENTER_T;
	}
}

// System pole allocation
void calc_gain(float *ke_x, float *ke_y,
				float *kv_x, float *kv_y)
{
float lx, ly;
	
	lx = contr_cam.p_x;
	ly = contr_cam.p_y;
	
	*kv_x = 2 * (lx - 1);
	*kv_y = 2 * (ly - 1);
	*ke_x = -(lx * lx - 2 * lx + 1) / TSCALE;
	*ke_y = -(ly * ly - 2 * ly + 1) / TSCALE;
	
}

//-----------------------------------------------------------------------------
// GRAPHIC FUNCTIONS
//-----------------------------------------------------------------------------

void show_status()
{
char s[6];
	
	sprintf(s, "%d", n);
	textout_ex(schermo, font, s, 
		XBASE_S + XINCR_S, YBASE_S, 14, -1);
	sprintf(s, "%1.0f", sine.amp);
	textout_ex(schermo, font, s, 
		XBASE_S + XINCR_S, YBASE_S + YINCR_S, 14, -1);
	sprintf(s, "%1.0f", sine.per);
	textout_ex(schermo, font, s, 
		XBASE_S + XINCR_S, YBASE_S + 2 * YINCR_S, 14, -1);
	show_dmiss();	
	sprintf(s, "%d", contr_cam.size);
	textout_ex(schermo, font, s, 
		XBASE_S + XINCR_S, YBASE_S + 7 * YINCR_S, 14, -1);
	sprintf(s, "%1.2f", contr_cam.a_x);
	textout_ex(schermo, font, s, 
		XBASE_S + XINCR_S, YBASE_S + 3 * YINCR_S, 14, -1);
	sprintf(s, "%1.2f", contr_cam.a_y);
	textout_ex(schermo, font, s, 
		XBASE_S + XINCR_S, YBASE_S + 4 * YINCR_S, 14, -1);
	sprintf(s, "%1.2f", contr_cam.p_x);
	textout_ex(schermo, font, s, 
		XBASE_S + XINCR_S, YBASE_S + 5 * YINCR_S, 14, -1);
	sprintf(s, "%1.2f", contr_cam.p_y);
	textout_ex(schermo, font, s, 
		XBASE_S + XINCR_S, YBASE_S + 6 * YINCR_S, 14, -1);	
}

// Print on schermo the total number of deadline misses.
void show_dmiss()
{
char s[6];
int tot_dmiss;

	tot_dmiss = target_tp.dmiss + graphic_tp.dmiss + ui_tp.dmiss +
					camera_tp.dmiss + acquisition_tp.dmiss;
	sprintf(s, "%d", tot_dmiss);
	textout_ex(schermo, font, s, 
		XBASE_S + XINCR_S, FTHLY - 20, 14, -1);
}

void draw()
{
	blit(bkg, schermo, 0, 0, 0, 0, XWIN, YWIN);
	sem_post(&sem1);
	draw_target();
	if (n == 3)	draw_trail();
	draw_sel();
}

//-----------------------------------------------------------------------------
// USER INTERFACE FUNCTIONS
//-----------------------------------------------------------------------------

// Increments the selected element.
void increment()
{
	switch (selection) {
		case 0 :
			pthread_mutex_lock(&mux_sel);
			if (n < 3) n++;
			if (n == 2) reinit_sine();
			pthread_mutex_unlock(&mux_sel);
			break;
		case 1 :
			pthread_mutex_lock(&mux_sine);
			if (sine.amp < AMP_MAX) sine.amp++;
			pthread_mutex_unlock(&mux_sine);
			break;
		case 2 :
			pthread_mutex_lock(&mux_sine);
			if (sine.per < PER_MAX)	{
				sine.per++;
				sine.ls = sine.ls + 1 / N;
			}
			pthread_mutex_unlock(&mux_sine);
			break;
		case 3 :
			if (contr_cam.a_x < A_MAX) contr_cam.a_x += A_STEP;
			break;
		case 4 :
			if (contr_cam.a_y < A_MAX) contr_cam.a_y += A_STEP;
			break;
		case 5 :
			if (contr_cam.p_x < P_MAX - 0.01) contr_cam.p_x += P_STEP;
			break;
		case 6 :
			if (contr_cam.p_y < P_MAX - 0.01) contr_cam.p_y += P_STEP;
			break;
		case 7 :
			pthread_mutex_lock(&mux_size);
			if (contr_cam.size < SMAX) contr_cam.size += S_STEP;
			pthread_mutex_unlock(&mux_size);
			break;
		default :
			break;
	}
}

// Decrements the selected element.
void decrement()
{
	switch (selection) {
		case 0 :
			pthread_mutex_lock(&mux_sel);
			if (n > 1) n--;
			if (n == 2) reinit_sine();
			pthread_mutex_unlock(&mux_sel);
			break;
		case 1 :
			pthread_mutex_lock(&mux_sine);
			if (sine.amp > AMP_MIN) sine.amp--;
			pthread_mutex_unlock(&mux_sine);
			break;
		case 2 :
			pthread_mutex_lock(&mux_sine);
			if (sine.per > PER_MIN)	{
				sine.per--;
				sine.ls = sine.ls - 1 / N;
			}
			pthread_mutex_unlock(&mux_sine);
			break;
		case 3 :
			if (contr_cam.a_x > A_MIN) contr_cam.a_x -= A_STEP;
			break;
		case 4 :
			if (contr_cam.a_y > A_MIN) contr_cam.a_y -= A_STEP;
			break;
		case 5 :
			if (contr_cam.p_x + 0.01 > P_MIN) contr_cam.p_x -= P_STEP;
			break;
		case 6 :
			if (contr_cam.p_y + 0.01 > P_MIN) contr_cam.p_y -= P_STEP;
			break;
		case 7 :
			pthread_mutex_lock(&mux_size);
			if (contr_cam.size > SMIN) contr_cam.size -= S_STEP;
			pthread_mutex_unlock(&mux_size);
			break;
		default :
			break;
	}
}

// Selects the way the target moves.
void select_movement()
{
	switch (c)	{
		case KEY_1_PAD :
			n = 1;
			break;
		case KEY_2_PAD :
			if (n == 2)	{
				sine.ang = ang_conv(45 * (rand() % 8));
			}
			else {
				n = 2;
				reinit_sine();
			}
			break;
		case KEY_3_PAD :
			bersaglio.v = V_MOUSE;
			mov.change = 1;
			n = 3;
			break;
		default:
			break;
	}
}	

//Selects and changes status parameters.
void select_change_param()
{
	switch (c) {
		case KEY_DOWN :
			if (selection < SEL_MAX) selection++;
			break;
		case KEY_UP :
			if (selection > SEL_MIN) selection--;
			break;
		case KEY_PLUS_PAD :
			increment();
			break;
		case KEY_MINUS_PAD :
			decrement();
			break;
		default:
			break;
	}
}

void reset()
{
	init_structures();
}

// Draws the selective sphere.
void draw_sel()
{
	pthread_mutex_lock(&mux_sel);
	circlefill(schermo, (XBASE_S + TRDLX) / 2,
		 YBASE_S + 3 + selection * YINCR_S, DIM_SEL, 14);
	pthread_mutex_unlock(&mux_sel);	
}

// Get mouse coordinates.
void get_coordinates(int x, int y)
{
	pthread_mutex_lock(&mux_mouse);
	mouse.x = x;
	mouse.y = y;
	mouse.happ = 1;
	pthread_mutex_unlock(&mux_mouse);
}

// Signal not left mouse's click.
void not_pressed()
{
	pthread_mutex_lock(&mux_mouse);
	mouse.happ = 0;
	pthread_mutex_unlock(&mux_mouse);
}

// To signal the left mouse's click and stores its coordinates.
void press_left_click()
{
int x, y;
int outl, outr, outt, outb;

	if (mouse_b & 1) {
		pthread_mutex_lock(&mux_mouse);
		x = mouse_x - FSTLX - 3;
		y = FTHLY - mouse_y + 5;
		pthread_mutex_unlock(&mux_mouse);
		outl = (x <= H * TRG_SCALE);					
		outr = (x >= SNDLX - FSTLX - H * TRG_SCALE);
		outt = (y >= FTHLY - TRDLY - H * TRG_SCALE);
		outb = (y <= H * TRG_SCALE);
		if (!(outt || outb ||							// No collision							
			outl || outr))	{
			get_coordinates(x, y);
		} else not_pressed();
	}
	else not_pressed();
}

//-----------------------------------------------------------------------------
// INIT FUNCTIONS
//-----------------------------------------------------------------------------

// Initialize Allegro graphics with black background and boxes.
void init_allegro()
{
	allegro_init();
	install_keyboard();
	install_mouse();
	show_os_cursor(MOUSE_CURSOR_ARROW);
	
	set_color_depth(8);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN, 0, 0);
}

// Initialize background.
void init_background()
{
	clear_to_color(screen, 0);
	rect(screen, FSTLX, FTHLY, SNDLX, TRDLY, 15);
	rect(screen, FSTLX, SNDLY, SNDLX, FSTLY, 15);
	rect(screen, TRDLX, FTHLY, FTHLX, FSTLY, 15);
}

//Initialize menu box.
void fill_menu()
{
	textout_ex(screen, font, "1:random",
		 XBASE_M, YBASE_M, 15, -1);
	textout_ex(screen, font, "2:sine wave",
		 XBASE_M, YBASE_M + YINCR_M ,15, -1);
	textout_ex(screen, font, "3:mouse", 
		XBASE_M + XINCR_M - K, YBASE_M, 15, -1);
	textout_ex(screen, font, "R:reset param", 
		XBASE_M + XINCR_M - K, YBASE_M + YINCR_M, 15, -1);
	textout_ex(screen, font, "A:amplitude", 
		XBASE_M + 2 * XINCR_M - K, YBASE_M, 15, -1);
	textout_ex(screen, font, "P:period", 
		XBASE_M + 2 * XINCR_M - K, YBASE_M + YINCR_M, 15, -1);
	textout_ex(screen, font, "X:x dynamic", 
		XBASE_M + 3 * XINCR_M - K, YBASE_M, 15, -1);
	textout_ex(screen, font, "Y:y dynamic", 
		XBASE_M + 3 * XINCR_M - K, YBASE_M + YINCR_M, 15, -1);
	textout_ex(screen, font, "S:cam size", 
		XBASE_M + 4 * XINCR_M - K, YBASE_M, 15, -1);
	textout_ex(screen, font, "ESC:exit", 
		XBASE_M + 4 * XINCR_M - K, YBASE_M + YINCR_M, 15, -1);
}

// Initialize status box.
void fill_status()
{
	textout_ex(screen, font, "Movement:", 
		XBASE_S, YBASE_S, 15, -1);
	textout_ex(screen, font, "A:", 
		XBASE_S, YBASE_S + YINCR_S, 15, -1);
	textout_ex(screen, font, "P:", 
		XBASE_S, YBASE_S + 2 * YINCR_S, 15, -1);
	textout_ex(screen, font, "X motor pole:", 
		XBASE_S, YBASE_S + 3 * YINCR_S, 15, -1);
	textout_ex(screen, font, "Y motor pole:", 
		XBASE_S, YBASE_S + 4 * YINCR_S, 15, -1);
	textout_ex(screen, font, "X system pole:", 
		XBASE_S, YBASE_S + 5 * YINCR_S, 15, -1);
	textout_ex(screen, font, "Y system pole:", 
		XBASE_S, YBASE_S + 6 * YINCR_S, 15, -1);
	textout_ex(screen, font, "S size:", 
		XBASE_S, YBASE_S + 7 * YINCR_S, 15, -1);
	textout_ex(screen, font, "Deadline misses:", 
		XBASE_S, FTHLY - 20, 15, -1);
}

// Init target thread parameters.
void init_target_thread(int i)
{	
	target_tp.period = TASK_PER;
	target_tp.deadline = TASK_PER;
	target_tp.priority = 25;
	target_tp.dmiss = 0;
	
	pthread_attr_init(&target_att);
	pthread_attr_setinheritsched(&target_att, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&target_att, i);
	
	target_pr.sched_priority = target_tp.priority;
	pthread_attr_setschedparam(&target_att, &target_pr);
	
	pthread_attr_setstacksize(&target_att, STK_SIZE);
}

// Initialize graphic thread.
void init_graphic_thread(int i)
{
	graphic_tp.period = TASK_PER;
	graphic_tp.deadline = TASK_PER;
	graphic_tp.priority = 25;
	graphic_tp.dmiss = 0;
	
	pthread_attr_init(&graphic_att);
	pthread_attr_setinheritsched(&graphic_att, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&graphic_att, i);
	
	graphic_pr.sched_priority = graphic_tp.priority;
	pthread_attr_setschedparam(&graphic_att, &graphic_pr);
	
	pthread_attr_setstacksize(&graphic_att, STK_SIZE);
}

// Initialize user interface thread.
void init_ui_thread(int i)
{
	ui_tp.period = 60;
	ui_tp.deadline = 60;
	ui_tp.priority = 25;
	ui_tp.dmiss = 0;
	
	pthread_attr_init(&ui_att);
	pthread_attr_setinheritsched(&ui_att, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&ui_att, i);
	
	ui_pr.sched_priority = ui_tp.priority;
	pthread_attr_setschedparam(&ui_att, &ui_pr);
	
	pthread_attr_setstacksize(&ui_att, STK_SIZE);
}

// Initialize camera thread.
void init_camera_thread(int i)
{
	camera_tp.period = TASK_PER;
	camera_tp.deadline = TASK_PER;
	camera_tp.priority = 25;
	camera_tp.dmiss = 0;
	
	pthread_attr_init(&camera_att);
	pthread_attr_setinheritsched(&camera_att, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&camera_att, i);
	
	camera_pr.sched_priority = camera_tp.priority;
	pthread_attr_setschedparam(&camera_att, &camera_pr);
	
	pthread_attr_setstacksize(&camera_att, STK_SIZE);
}

void init_acquisition_thread(int i)
{
	acquisition_tp.period = TASK_PER;
	acquisition_tp.deadline = TASK_PER;
	acquisition_tp.priority = 25;
	acquisition_tp.dmiss = 0;
	
	pthread_attr_init(&acquisition_att);
	pthread_attr_setinheritsched(&acquisition_att, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&acquisition_att, i);
	
	acquisition_pr.sched_priority = acquisition_tp.priority;
	pthread_attr_setschedparam(&acquisition_att, &acquisition_pr);
	
	pthread_attr_setstacksize(&acquisition_att, STK_SIZE);
}

// Initialize all threads.
void init_threads(int i)
{
	init_target_thread(i);
	init_graphic_thread(i);
	init_ui_thread(i);
	init_camera_thread(i);
	init_acquisition_thread(i);
}

// Initialize global structures of target, camera, motors.
void init_structures()
{
	init_target();
	init_trail();
	init_mouse();
	init_mov();
	init_sine();
	init_cam();
	init_contr_cam();
}

// To set one mutex.
void set_mutex(int i, pthread_mutex_t *mux, pthread_mutexattr_t *matt)
{
	pthread_mutexattr_init(matt);
	pthread_mutexattr_setprotocol(matt, i);
	pthread_mutex_init(mux, matt);
}

// To init all mutex.
void init_mutex(int i)
{
	set_mutex(i, &mux_target, &matt_target);
	set_mutex(i, &mux_trail, &matt_trail);
	set_mutex(i, &mux_mouse, &matt_mouse);
	set_mutex(i, &mux_mov, &matt_mov);
	set_mutex(i, &mux_sine, &matt_sine);
	set_mutex(i, &mux_sel, &matt_sel);
	set_mutex(i, &mux_cam, &matt_cam);
	set_mutex(i, &mux_size, &matt_size);
	set_mutex(i, &mux_polex, &matt_polex);
	set_mutex(i, &mux_poley, &matt_poley);
	set_mutex(i, &mux_schermo, &matt_schermo);
}

void init_sem()
{
	sem_init(&sem1, 0, 0);
	sem_init(&sem2, 0, 0);
}

// Initialize Allegro graphics and background.
void init()
{
	init_allegro();
	init_background();
	fill_menu();
	fill_status();
	init_structures();
	init_threads(SCHED_FIFO);
	init_mutex(PTHREAD_PRIO_INHERIT);
	init_sem();
	bkg = create_bitmap(XWIN, YWIN);
	blit(screen, bkg, 0, 0, 0, 0, XWIN, YWIN);	// bkg stores background
	schermo = create_bitmap(XWIN, YWIN);
}

//-----------------------------------------------------------------------------
// THREAD FUNCTIONS
//-----------------------------------------------------------------------------

// Create all threads.
void create_tasks()
{
	pthread_create(&acquisition_id, &acquisition_att,
					 acquisition_task, &acquisition_tp);
	pthread_create(&target_id, &target_att, target_task, &target_tp);
	pthread_create(&camera_id, &camera_att, camera_task, &camera_tp);
	pthread_create(&ui_id, &ui_att, ui_task, &ui_tp);
	pthread_create(&graphic_id, &graphic_att, graphic_task, &graphic_tp);
}

// Wait user presses ESC to quit all threads.
void wait_for_task_end()
{
	pthread_join(graphic_id, NULL);
	pthread_join(target_id, NULL);
	pthread_join(ui_id, NULL);
	pthread_join(camera_id, NULL);
	pthread_join(acquisition_id, NULL);
}

//-----------------------------------------------------------------------------
// TASK FUNCTIONS
//-----------------------------------------------------------------------------

// Target task.
void *target_task(void *p)
{
struct task_par *tp;

	tp = (struct task_par *)p;
		
	set_period(tp);
	
	while (!end)	{
		switch (n) {
			case 1 :
				movement_random();
				break;
			case 2 :
				movement_sine();
				break;
			case 3 :
				movement_mouse();
				break;
			default : break;
		}
		deadline_miss(tp);
		wait_for_period(tp);
	}	
} 

// Graphic task.
void *graphic_task(void *p)
{
struct task_par *tp;

	tp = (struct task_par *)p;
	
	set_period(tp);
	
	while (!end)	{
		draw();					
		show_status();
		sem_wait(&sem2);
		blit(schermo, screen, 0, 0, 0, 0, XWIN, YWIN);	
				wait_for_period(tp);
		deadline_miss(tp);
	}	
}

// User interface task.
void *ui_task(void *p)
{
struct task_par *tp;

	tp = (struct task_par *)p;
	
	set_period(tp);
	
	do	{
		c = get_scancode();
		if (c == KEY_R) reset();
		select_movement();
		select_change_param();
		if (n == 3) press_left_click();
		deadline_miss(tp);
		wait_for_period(tp);	
	} while (c != KEY_ESC);
	
	sem_post(&sem1);
	sem_post(&sem2);
	end = 1;
}

// Camera task.
void *camera_task(void *p)
{
float xc, yc, xc_old, yc_old, ax, ay, x, y;
float ud_x, ut_x, ud_y, ut_y, lpx, lpy, px, py;
float ke_x, ke_y, kv_x, kv_y;
int s;

struct task_par *tp;

	tp = (struct task_par *)p;
	
	set_period(tp);
	
	while (!end)	{
		update_local_variable_control(&s,
							&xc, &yc, &xc_old, &x,
							&ax, &ay, &yc_old, &y);
		if (contr_cam.not_found) {
			handle_not_found(s);
		} else {
			compute_reference(xc, yc, xc_old, yc_old);
		}
		if (distanza(contr_cam.rif_x, contr_cam.rif_y,
					 					x, y) >= CAM_TOLL)	{
			calc_gain(&ke_x, &ke_y, &kv_x, &kv_y);
			ud_x = ke_x * (-contr_cam.rif_x + x) + kv_x * cam.vx;
			ud_y = ke_y * (-contr_cam.rif_y + y) + kv_y * cam.vy;
			lpx = exp(-ax * TSCALE);
			lpy = exp(-ay * TSCALE);
			ut_x = lpx * contr_cam.ut_x + (1 - lpx) * ud_x;
			ut_y = lpy * contr_cam.ut_y + (1 - lpy) * ud_y;
			update_cam_state(ut_x, ut_y, s);
		}
		deadline_miss(tp);
		wait_for_period(tp);
	}
}

// Acquisition task.
void *acquisition_task(void *p)
{
float xc, yc;
int s;
struct task_par *tp;

	tp = (struct task_par *)p;
	
	set_period(tp);
	
	while (!end)	{
		init_image();
		update_local_variable_acq(&xc, &yc, &s);
		get_put_image(xc, yc, s);
		calc_centroid(xc, yc, s);
		deadline_miss(tp);
		wait_for_period(tp);
	}
}


//-----------------------------------------------------------------------------
// MAIN
//-----------------------------------------------------------------------------

int main(int argc, char *argv[])
{
	init();					// Initialize Allegro graphics and background.
	create_tasks();			// Create all threads.
	wait_for_task_end();	// Wait user presses ESC to quit all threads.
	allegro_exit();			// Close Allegro.
	return 0;
}
