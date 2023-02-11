/*
 * This file is an header file which contains auxiliary
 * functions and structures useful to manage threads.
 */

#ifndef THREAD_FUNCTIONS_H
#define THREAD_FUNCTIONS_H

#include <time.h>

void time_copy(struct timespec *td,
			   struct timespec ts);
			   
void time_add_ms(struct timespec *t, int ms);

int time_cmp(struct timespec t1,
			 struct timespec t2);
			 
struct task_par	{
	int arg;				// task argument
	long wcet;				// in microseconds
	int period;				// in milliseconds
	int deadline;			// relative (ms)
	int priority;			// in [0,99]
	int dmiss;				// misses counter
	struct timespec at;		// next activation time
	struct timespec dl;		// absolute deadline
};

void set_period(struct task_par *tp);

void wait_for_period(struct task_par *tp);

int deadline_miss(struct task_par *tp);

#endif
