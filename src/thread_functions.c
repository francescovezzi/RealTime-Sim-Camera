#include "thread_functions.h"
#include <time.h>
#include <pthread.h>

// Copies ts in the variable pointed by td.
void time_copy(struct timespec *td,
			   struct timespec ts)
{
	td->tv_sec = ts.tv_sec;
	td->tv_nsec = ts.tv_nsec;
}

// Adds a value ms expressed in milliseconds to the time variable pointed by t.
void time_add_ms(struct timespec *t, int ms)
{
	t->tv_sec += ms / 1000;
	t->tv_nsec += (ms % 1000) * 1000000;
	if (t->tv_nsec >= 1000000000) {
		t->tv_sec += 1;
		t->tv_nsec -= 1000000000;
	}
}

// Returns 1 if t1 > t2, -1 if t1 < t2, else 0.
int time_cmp(struct timespec t1,
			 struct timespec t2)
{
	if (t1.tv_sec > t2.tv_sec)	return 1;
	if (t1.tv_sec < t2.tv_sec)	return -1;
	if (t1.tv_nsec > t2.tv_nsec)	return 1;
	if (t1.tv_nsec < t2.tv_nsec)	return -1;
	return 0;
}

//-----------------------------------------------------------------------------
// Reads the current time and computes the next activation time and the absolute 
// deadline of the task.
//-----------------------------------------------------------------------------
void set_period(struct task_par *tp)
{
struct timespec t;
	
	clock_gettime(CLOCK_MONOTONIC, &t);
	time_copy(&(tp->at), t);
	time_copy(&(tp->dl), t);
	time_add_ms(&(tp->at), tp->period);
	time_add_ms(&(tp->dl), tp->deadline);
}
//-----------------------------------------------------------------------------
// Suspends the calling thread until the next activation and, when awaken,
// updates activation time and deadline.
//-----------------------------------------------------------------------------
void wait_for_period(struct task_par *tp)
{
	clock_nanosleep(CLOCK_MONOTONIC,
			TIMER_ABSTIME, &(tp->at), NULL);
	time_add_ms(&(tp->at), tp->period);
	time_add_ms(&(tp->dl), tp->deadline);
}

// Detects deadline miss, increments value of dmiss and returns 1.
int deadline_miss(struct task_par *tp)
{
struct timespec now;
	
	clock_gettime(CLOCK_MONOTONIC, &now);
	
	if (time_cmp(now, tp->dl) > 0)	{
		tp->dmiss++;
		return 1;
	}
	return 0;
}
