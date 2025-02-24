#pragma once
#include <pico/mutex.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
  unsigned int reader_count;
  mutex_t reader_mutex;
  mutex_t writer_mutex;
} shared_mutex_t;

void shared_mutex_init(shared_mutex_t* s_mtx);

void shared_mutex_enter_blocking_shared(shared_mutex_t* s_mtx);
void shared_mutex_exit_shared(shared_mutex_t* s_mtx);

void shared_mutex_enter_blocking_exclusive(shared_mutex_t* s_mtx);
void shared_mutex_exit_exclusive(shared_mutex_t* s_mtx);

#ifdef __cplusplus
}
#endif
