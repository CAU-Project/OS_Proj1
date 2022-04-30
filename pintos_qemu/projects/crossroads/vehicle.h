#ifndef __PROJECTS_PROJECT1_VEHICLE_H__
#define __PROJECTS_PROJECT1_VEHICLE_H__

#include "projects/crossroads/position.h"
#include "threads/synch.h"

#define VEHICLE_STATUS_READY 	0
#define VEHICLE_STATUS_RUNNING	1
#define VEHICLE_STATUS_FINISHED	2

struct vehicle_info {
	char id;
	char state;
	char start;
	char dest;
	struct position position;
	struct lock **map_locks;
};

void vehicle_loop(void *vi);
void init_on_mainthread(int thread_cnt);

/* One semaphore in a list. */
struct semaphore_elem 
  {
    struct list_elem elem;              /* List element. */
    struct semaphore semaphore;         /* This semaphore. */
  };


/* Condition variable. */
struct condition2 
  {
    struct list waiters;        /* List of waiting threads. */
  };



void cond2_init (struct condition2 *);
void cond2_wait (struct condition2 *);
void cond2_signal (struct condition2 *);
void cond2_broadcast (struct condition2 *);


int is_position_intersection(struct position);
bool can_enter_intersection(int start, int dest);
int get_intersection_idx(struct position);
void before_lock_wait(void);

#endif /* __PROJECTS_PROJECT1_VEHICLE_H__ */