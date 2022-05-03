
#include <stdio.h>

#include "threads/malloc.h"
#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"


/* project1 */
struct semaphore *y;		//교차로 로직 관련된 임계영역을 보호. 동시에 판단하지 못하도록
struct semaphore *x;		//remain_count를 보호하기 위한 semaphore
struct condition2 *cond;	//하나의 턴에서 성공적으로 이동을 끝낸 쓰레드들을 관리하기 위한 상태 변수
int flag;					//Special Case를 처리하기 위한 flag. 만약 마지막 쓰레드가 block 되는 경우에 처리를 한다.
int lock_wait;				//lock에서 block 된 후, wake up 한 쓰레드들을 다음 턴에는 움직이도록 처리하는 변수
int avilable_move_count;	//다음 턴에 몇개의 쓰레드를 체크해야 할 지를 저장하는 변수
int remain_count;			//현재 턴에서 몇개의 쓰레드가 더 이동해야 할지를 저장하는 변수

// 교차로의 잠김 유무를 체크하기 위한 배열
bool intersection_locked[8]={
	0,0,0,0,0,0,0,0
};


/*synch.c 파일에 있는 condition variable을 수정*/
void cond2_init (struct condition2 *cond)
{
  ASSERT (cond != NULL);

  list_init (&cond->waiters);
}

/*cond2_wait(cond)로 해당 함수를 호출한 쓰레드를 block*/
void cond2_wait (struct condition2 *cond) 
{
  struct semaphore_elem waiter;

  ASSERT (cond != NULL);
  
  sema_init (&waiter.semaphore, 0);
  list_push_back (&cond->waiters, &waiter.elem);
  sema_down (&waiter.semaphore);
}

void cond2_signal (struct condition2 *cond) 
{
  ASSERT (cond != NULL);

  if (!list_empty (&cond->waiters)) 
    sema_up (&list_entry (list_pop_front (&cond->waiters),
                          struct semaphore_elem, elem)->semaphore);
}

/*cond2_wait(cond)로 지금 주어진 condition variable에 의해서 잠든 모든 쓰레드를 깨운다.*/
void cond2_broadcast (struct condition2 *cond) 
{
  ASSERT (cond != NULL);

  while (!list_empty (&cond->waiters))
    cond2_signal (cond);
}

/*교차로의 8개의 좌표들*/
const struct position intersection_area[8] = {
	{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2}
};

/* path. A:0 B:1 C:2 D:3 */
/*[start][dest]에 따른 교차로에서의 좌표*/
const struct position intersection_path[4][4][10] = {
	/* from A */ {
		/* to A */
		{0,0},
		/* to B */
		{{4,2},},
		/* to C */
		{{4,2},{4,3},{4,4},},
		/* to D */
		{{4,2},{4,3},{4,4},{3,4},{2,4},}
	},
	/* from B */ {
		/* to A */
		{{4,4},{3,4},{2,4},{2,3},{2,2},},
		/* to B */
		{0,0},
		/* to C */
		{{4,4},},
		/* to D */
		{{4,4},{3,4},{2,4},}
	},
	/* from C */ {
		/* to A */
		{{2,4},{2,3},{2,2},},
		/* to B */
		{{2,4},{2,3},{2,2},{3,2},{4,2},},
		/* to C */
		{0,0},
		/* to D */
		{{2,4},}
	},
	/* from D */ {
		/* to A */
		{{2,2},},
		/* to B */
		{{2,2},{3,2},{4,2},},
		/* to C */
		{{2,2},{3,2},{4,2},{4,3},{4,4},},
		/* to D */
		{0,0}
	}
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

/*[start][dest]에 따라 교차로에서 잠궈야 하는 좌표의 개수 반환*/
const int num_intersection_hold[4][4]={
	{0,1,3,5},
	{5,0,1,3},
	{3,5,0,1},
	{1,3,5,0}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}
/*주어진 좌표가 교차로 내의 좌표인지 확인*/
int is_position_intersection(struct position pos)
{
	int result=0;
	int i;
	for(i=0;i<8;i++){
		result += (pos.row == intersection_area[i].row  &&  pos.col == intersection_area[i].col);
	}
	return result;
}

/*[start][dest]로 가는 차가 교차로 진입 전에 진입 가능 여부를 확인하는 함수*/
bool can_enter_intersection(int start, int dest){
	sema_down(y);
	int num = num_intersection_hold[start][dest];
	int i=0;
	bool result = true;
	for(i=0; i<num; i++){
		struct position pos = intersection_path[start][dest][i];
		int pos_idx = get_intersection_idx(pos);
		if(intersection_locked[pos_idx] == 1){
			result = false;
			break;
		}
	}
	return result;
}

int get_intersection_idx(struct position pos){
	int i =0;
	for(i=0;i<8;i++){
		if(pos.col == intersection_area[i].col && pos.row == intersection_area[i].row){
			return i;
		}
	}
	return 0;
}

/*thread를 block 하기 전에 처리하는 함수*/
void before_lock_wait(){
	sema_down(x);
	remain_count -= 1;
	avilable_move_count -= 1;
	if(remain_count == 0){
		cond2_signal(cond);
		flag = 1;
	} 
	sema_up(x);
}

/*try move는 크게 3가지 case로 구성된다.
1. 현재 좌표가 교차로 내에 있는 경우
 1.1 만약 다음 좌표가 교차로를 벗어난다면, 교차로 진입시 잠궜던 경로 모두 release
 1.2 만약 교차로를 벗어나지 않는다면 한칸 전진
2. 현재 좌표가 교차로는 아니지만, 다음 칸에 교차로에 진입하는 경우
 2.1 내가 진입하려는 좌표를 교차로 내에서 다른 차가 점유하고 있는 경우. return 2 -> 아무런 행위 없이 턴 종료
 2.2 내가 진입하려는 좌표가 비어있는 경우, 내가 가야하는 모든 경로를 잠그고 이동
3. 현재 좌표가 교차로는 아니면서, 다음 칸에 교차로 진입도 아닌 경우
 3.1 다음 칸에 다른 차가 점유(lock)하고 있다면, 턴을 종료
 3.2 다음 칸이 비어 있다면 한칸 이동
*/
/* return 0:termination, 1:success, 2:wait intersection */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;

	pos_next = vehicle_path[start][dest][step];
	pos_cur = vi->position;
	
	/*aAA,bBB 같은 쓰레드 처리*/
	if(start == dest){ return 0; }

	if(is_position_intersection(pos_cur)){
		/*1. 현재 좌표가 교차로 내에 있을 때 처리*/
		vi->position = pos_next;

		/*1.1 현재 교차로 내에 있는데, 다음에 교차로를 벗어나면, 자신이 lock한 교차로 좌표 모두 release*/
		if(!is_position_intersection(vi->position)){
			lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
			sema_down(y);
			int num = num_intersection_hold[start][dest];
			int i;
			for(i=0; i<num; i++){
				struct position pos = intersection_path[start][dest][i];
				int pos_idx = get_intersection_idx(pos);
				intersection_locked[pos_idx] = false;
				lock_release(&vi->map_locks[pos.row][pos.col]);		
			}
			sema_up(y);
		}
		/*1.2 이동만 하고 return 1*/
		return 1;	
	}
	else{
		/*2. 현재 교차로 안에 있지 않은 상태*/
		if(is_position_intersection(pos_next)){
			if(can_enter_intersection(start,dest)){
				/*2.1 먼저 교차로에 진입 가능한지 판단해서 진입 가능하다면, 모두 lock*/
				int num = num_intersection_hold[start][dest];
				int i;
				for(i=0; i<num; i++){
					struct position pos = intersection_path[start][dest][i];
					int pos_idx = get_intersection_idx(pos);
					intersection_locked[pos_idx] = true;
					lock_acquire(&vi->map_locks[pos.row][pos.col]);					
				}
				sema_up(y);
				vi->position = pos_next;
				lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
				return 1;
			}else{/*2.2 교차로에 다른 차량이 점유하고 있다면, return 2로 이번 단위스탭에선 아무런 움직임 x*/
				sema_up(y);
				return 2;
			}

		}
		else{/*3.현재 교차로에 없으면서, 다음 좌표도 교차로가 아닌 상태*/

			/*차량이 맵을 벗어나서 끝내는 경우*/
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

			if(!lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])){
				/*3.1 다음 칸에 다른 차가 점유(lock)하고 있다면, 턴을 종료 후 block*/
				before_lock_wait();
				lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
				sema_down(x);
				remain_count +=1;
				lock_wait++;
				sema_up(x);			
			}

			/*3.2 다음 칸이 비어 있다면 한칸 이동*/
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
	}
}

void init_on_mainthread(int thread_cnt){
	/* project1 */
	/* Called once before spawning threads */
	avilable_move_count = thread_cnt;	
	remain_count = avilable_move_count;

	x = malloc(sizeof (struct semaphore));
	sema_init(x,1);

	y = malloc(sizeof (struct semaphore));
	sema_init(y,1);

	cond = malloc(sizeof (struct condition2));
	cond2_init(cond);

	flag = 0;
	lock_wait = 0;
}

void vehicle_loop(void *_vi)
{
	int res;
	int start, dest, step;

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

		/*remain_count를 동시에 여러 쓰레드가 접근하지 못하도록 semaphore x 로 보호*/
		sema_down(x);
		remain_count -= 1;

		if(remain_count > 0){
			/*remain count가 양수인 경우, cond2_wait(cond)로 해당 쓰레드 block*/
			sema_up(x);

			/* sleep until remain_count = 0 */
			cond2_wait(cond);

		}else{
			/*remain count가 0인 경우, unitstep을 변경하고, 모든 쓰레드를 깨운다.*/
			crossroads_step += 1;
			unitstep_changed();

			/* wake up other wait thread*/		
			cond2_broadcast(cond);

			/*수치를 보정한다.*/
			avilable_move_count += lock_wait; //현재 턴에서 이동한 쓰레드로 인해서 block이 풀린 쓰레드는 다음턴에 이동 가능하도록 수 ++
			remain_count = avilable_move_count;
			lock_wait = 0;
			sema_up(x);
		}

		// 마지막 Thread가 block 되는 경우, try move에서 리턴값을 받지 못한다. 이 경우를 처리하기 위해서 flag를 도입.
		/*special case 1*/
		if(flag == 1){
			sema_down(x);
			crossroads_step += 1;
			unitstep_changed();

			/* wake up other wait thread*/		
			cond2_broadcast(cond);
			avilable_move_count += lock_wait;
			remain_count = avilable_move_count;
			lock_wait = 0;
			flag = 0;
			sema_up(x);
			}


		/* termination condition. */ 
		if (res == 0) {
			break;
		}

	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
