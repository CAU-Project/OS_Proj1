
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

/* project1 */
struct semaphore *y;	//not yet moved car
struct semaphore *x;
struct condition2 *cond;
int a;
int flag;
int lock_wait;
int avilable_move_count;
int vehicle_count;
int remain_count;
int wait_count;



void cond2_init (struct condition2 *cond)
{
  ASSERT (cond != NULL);

  list_init (&cond->waiters);
}

void cond2_wait (struct condition2 *cond) 
{
  struct semaphore_elem waiter;

  ASSERT (cond != NULL);
  ASSERT (!intr_context ());
  
  sema_init (&waiter.semaphore, 0);
  list_push_back (&cond->waiters, &waiter.elem);
  sema_down (&waiter.semaphore);
}

void cond2_signal (struct condition2 *cond) 
{
  ASSERT (cond != NULL);
  ASSERT (!intr_context ());

  if (!list_empty (&cond->waiters)) 
    sema_up (&list_entry (list_pop_front (&cond->waiters),
                          struct semaphore_elem, elem)->semaphore);
}

void cond2_broadcast (struct condition2 *cond) 
{
  ASSERT (cond != NULL);

  while (!list_empty (&cond->waiters))
    cond2_signal (cond);
}

const struct position critical_path[8] = {
	{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2}
};

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][10] = {
	/* from A */ {
		/* to A */
		{{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
		/* to B */
		{{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
		/* to C */
		{{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
		/* to D */
		{{-1,-1},}
	}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

static int is_position_critical(struct position pos)
{
	int result=0;
	int i;
	for(i=0;i<8;i++){
		result += (pos.row == critical_path[i].row  &&  pos.col == critical_path[i].col);
	}
	return result;
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;
	bool in_critical = false;

	pos_next = vehicle_path[start][dest][step];
	pos_cur = vi->position;

	if(is_position_critical(pos_cur)) in_critical = true;


	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			return 0;
		} 	
	}

	/* lock next position */
	if(lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])){
	}else{
		/*if lock try return false, then lock.*/
		sema_down(x);
		remain_count -= 1;
		avilable_move_count -= 1;
		if(remain_count == 0){
			cond2_signal(cond);
			flag = 1;
		} 
		sema_up(x);
		printf("\n\n\nLock down is start for id : %d",vi->id);
		lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
		printf("\n\n\nLock down is finished for id : %d",vi->id);
		lock_wait++;
		sema_down(x);
		printf("\n\n\nsema_down is started/finish for id : %d",vi->id);
		sema_up(x);
	}

	if(is_position_critical(pos_next)){
		
		
	}

	if (vi->state == VEHICLE_STATUS_READY) {
		/* start this vehicle */
		vi->state = VEHICLE_STATUS_RUNNING;
	} else {
		/* release current position */
		lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
	}
	/* update position */
	vi->position = pos_next;
	
	return 1;
}

void init_on_mainthread(int thread_cnt){
	/* project1 */
	/* Called once before spawning threads */
	vehicle_count = thread_cnt;
	avilable_move_count = vehicle_count;
	remain_count = avilable_move_count;

	x= malloc(sizeof (struct semaphore));
	sema_init(x,1);

	cond = malloc(sizeof (struct condition2));
	cond2_init(cond);

	
	wait_count = 0;
	flag = 0;
	lock_wait = 0;

	printf("vehicle count = %d",vehicle_count);

}

void vehicle_loop(void *_vi)
{
	int res;
	int start, dest, step;
	int i;

	struct vehicle_info *vi = _vi;

	start = vi->start - 'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;

	step = 0;
	while (1) {
		/* vehicle main code */
		res = try_move(start, dest, step, vi);
		if (res == 1) {
			step++;
		}
		/* termination condition. */ 
		if (res == 0) {
			avilable_move_count -= 1;
		}
		
		/* project1 */
		/* check all of vehicle moved */
		sema_down(x);
		remain_count -= 1;

		printf("\n\n\n\n\n\n\nremain_count : %d, vi : %d\n",remain_count,vi->id);
		if(remain_count > 0){
			wait_count +=1;
			sema_up(x);
			/* sleep until remain_count = 0 */
			
			cond2_wait(cond);

		}else{
			printf("wake up is started by id : %d\n",vi->id);
			/* cars moved finished. unitstep change! */
			crossroads_step += 1;
			unitstep_changed();

			/* wake up other wait thread*/		
			cond2_broadcast(cond);

			remain_count = avilable_move_count;
			sema_up(x);
			printf("wake up finished id : %d\n",vi->id);

		}

		/*special case 1*/
		if(flag == 1){
			sema_down(x);
			crossroads_step += 1;
			unitstep_changed();

			/* wake up other wait thread*/		
			cond2_broadcast(cond);
			remain_count = avilable_move_count;
			flag = 0;
			sema_up(x);
			}

		if(lock_wait > 0){
			remain_count += lock_wait;
			avilable_move_count += lock_wait;
			lock_wait = 0;
		}

		/* termination condition. */ 
		if (res == 0) {
			break;
		}

	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
