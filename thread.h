#ifndef _THREAD_H_
#define _THREAD_H_

#include <pthread.h>
#include "state.h"

#define WORK_SIZE (1024 * 10)
#define CREW_SIZE 4

typedef int (*criterion_p)(state *s, criteria *c);
typedef int (*calc_state_p)(state *s, state *pre_state);

typedef struct _work {
	int state_count;
	state *states;
	criterion_p criterion_fun;
	struct _work *next;
	int max_state;
	int valid;
} work_t, *work_p;

typedef struct _worker {
	int index;
	pthread_t thread;
	struct _crew *crew;
} worker_t, *worker_p;

typedef struct _crew {
	int crew_size;
	worker_t crew[CREW_SIZE];
	long capacity;
	long work_count;
	work_p first, last;
	calc_state_p calc_state_fun;
	int valid;
	state pre_state;
	state best_state;
	pthread_mutex_t mutex;
	pthread_cond_t go;
	pthread_cond_t ready;
	pthread_cond_t done;
} crew_t, *crew_p;

#ifdef DEBUG
#define log(arg...) printf(arg)
#else
#define log(arg...)
#endif

crew_t *create_crew(int crew_size, int capacity, calc_state_p calc_state_fun, state *pre_state);
int insert_work(crew_p crew, int state_count, state *states, criterion_p criterion_fun);
int reset_crew(crew_p crew, state *pres);
state *done(crew_p crew);

#endif /* _THREAD_H_ */
