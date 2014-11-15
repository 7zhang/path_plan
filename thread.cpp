#include <string.h>
#include "thread.h"

void *work_routine(void *arg);

crew_t *create_crew(int crew_size, int capacity, calc_state_p calc_state_fun, state *pre_state)
{
	crew_t *crew = (crew_t *)malloc(sizeof(crew_t));
	if (crew == NULL) {
		printf("malloc error!\n");
		return NULL;
	}
	int status;
	if (crew_size > CREW_SIZE) {
		printf("use max crew size %d\n", CREW_SIZE);
		crew_size = CREW_SIZE;
	}

	crew->crew_size = crew_size;
	crew->capacity = capacity;
	crew->work_count = 0;
	crew->first = NULL;
	crew->last = NULL;
	crew->calc_state_fun = calc_state_fun;
	crew->best_state.c.j = 0;
	crew->valid = 0;
	memcpy(&crew->pre_state, pre_state, sizeof(state));

	status = pthread_mutex_init(&crew->mutex, NULL);
	if (status != 0) {
		printf("init crew mutex error\n");
		return NULL;		
	}
	
	status = pthread_cond_init(&crew->go, NULL);
	if (status != 0) {
		printf("init crew condition variable go error\n");
		return NULL;
	}

	status = pthread_cond_init(&crew->ready, NULL);
	if (status != 0) {
		printf("init crew condition variable ready error\n");
		return NULL;
	}

	status = pthread_cond_init(&crew->done, NULL);
	if (status != 0) {
		printf("init crew condition variable done error\n");
		return NULL;
	}	

	for (int index = 0; index < crew->crew_size; index++) {
		crew->crew[index].index = index;
		crew->crew[index].crew = crew;
		status = pthread_create(&crew->crew[index].thread, NULL, work_routine, (void *)&crew->crew[index]);
		
		if (status != 0) {
			printf("init pthread_create error: thread %d\n", index);
			return NULL;
		}

		log("thread %d index started\n", index);
	}

	return crew;
}

void *work_routine(void *arg)
{
	worker_p me = (worker_p)arg;
	crew_p crew = me->crew;
	work_p work;
	int status;

	status = pthread_mutex_lock(&crew->mutex);
	if (status != 0) {
		printf("thread %d: error pthread_mutex_lock\n", me->index);
		return NULL;
	}

	while (1) {
		while (crew->first == NULL) {
			status = pthread_cond_wait(&crew->go, &crew->mutex);

			if (status != 0) {
				printf("thread %d: error pthread_cond_wait\n", me->index);
				return NULL;
			}

			log("thread %d blocked\n", me->index);
		}

		work = crew->first;
		crew->first = work->next;

		if (crew->first == NULL)
			crew->last == NULL;

		status = pthread_mutex_unlock(&crew->mutex);
		if (status != 0) {
				printf("thread %d: error pthread_mutex_unlock\n", me->index);
				return NULL;
		}

		for (int index = 0; index < work->state_count; index++) {
			if (crew->calc_state_fun == NULL) {
				printf("thread %d: error calc_state == NULL\n", me->index);
				return NULL;
			}

			state *s = &work->states[index];
			if (crew->calc_state_fun(s, &crew->pre_state)) {
				log("thread %d: error calc_state\n", me->index);
				continue;
			}

			if (work->criterion_fun(s, &s->c)) {
				log("thread %d: error criterion\n", me->index);
				continue;
			}

			work->valid = 1;
			if (s->c.j > work->states[work->max_state].c.j) {
				work->max_state = index;
			}
		}

		status = pthread_mutex_lock(&crew->mutex);
		if (status != 0) {
			printf("thread %d: error pthread_mutex_lock\n", me->index);
			return NULL;
		}

		if (work->valid == 1 && crew->best_state.c.j < work->states[work->max_state].c.j) {
			memcpy(&crew->best_state, &work->states[work->max_state], sizeof(state));
			crew->valid = 1;
		}

		crew->work_count--;

		if (crew->work_count < crew->capacity) {
			status = pthread_cond_signal(&crew->ready);
			if (status != 0) {
				printf("thread %d: error pthread_cond_signal crew ready\n", me->index);
				return NULL;
			}

			if (crew->work_count == 0) {
				status = pthread_cond_signal(&crew->done);
				if (status != 0) {
					printf("thread %d: error pthread_cond_signal crew done\n", me->index);
					return NULL;
				}
			}
		}

		free(work->states);
		free(work);
	}
}

int insert_work(crew_p crew, int state_count, state *states, criterion_p criterion_fun)
{
	int status;
	work_p work= (work_p)malloc(sizeof(work_t));
	if (work == NULL) {
		printf("insert_work: malloc error!\n");
		return -1;
	}

	work->state_count = state_count;
	work->states = states;
	work->criterion_fun = criterion_fun;
	work->next = NULL;
	work->max_state = 0;
	work->valid = 0;

	status = pthread_mutex_lock(&crew->mutex);
	if (status != 0) {
		printf("insert_work: error pthread_mutex_lock\n");
		return -1;
	}
	
	while (crew->work_count >= crew->capacity) {
		status = pthread_cond_wait(&crew->ready, &crew->mutex);
		if (status != 0) {
			printf("insert_work: error pthread_cond_wait crew ready\n");
			return -1;
		}
	}

	if (crew->first == NULL) {
		crew->first = work;
		crew->last = work;
	} else {
		crew->last->next = work;
		crew->last = work;
	}

	crew->work_count++;
	status = pthread_cond_signal(&crew->go);
	if (status != 0) {
		printf("insert_work: error pthread_cond_signal crew go\n");
		return -1;
	}
	
	status = pthread_mutex_unlock(&crew->mutex);
	if (status != 0) {
		printf("insert_work: error pthread_mutex_lock\n");
		return -1;
	}

	return 0;
}

int reset_crew(crew_p crew, state *pres)
{
	int status;

	status = pthread_mutex_lock(&crew->mutex);
	if (status != 0) {
		printf("done: error pthread_mutex_lock\n");
		return -1;
	}

	crew->first = NULL;
	crew->last = NULL;
	memcpy(&crew->pre_state, pres, sizeof(state));
	crew->best_state.c.j = 0;
	crew->valid = 0;
	
	status = pthread_mutex_unlock(&crew->mutex);
	if (status != 0) {
		printf("done: error pthread_mutex_unlock\n");
		return -1;
	}

	return 0;
}

state *done(crew_p crew)
{
	int status;

	status = pthread_mutex_lock(&crew->mutex);
	if (status != 0) {
		printf("done: error pthread_mutex_lock\n");
		return NULL;
	}
	
	while (crew->work_count > 0) {
		status = pthread_cond_wait(&crew->done, &crew->mutex);
		if (status != 0) {
			printf("insert_work: error pthread_cond_wait crew done\n");
			return NULL;
		}
	}

	state *ret = NULL;
	if (crew->valid == 0)
		goto label;
	ret = (state *)malloc(sizeof(state));
	if (ret == NULL) {
		printf("done: malloc error!\n");
		return NULL;
	}

	memcpy(ret, &crew->best_state, sizeof(state));

label:	status = pthread_mutex_unlock(&crew->mutex);
	if (status != 0) {
		printf("done: error pthread_mutex_unlock\n");
		if (ret != NULL)
			free(ret);
		return NULL;
	}

	return ret;
}
