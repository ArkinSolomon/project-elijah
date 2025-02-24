#include "shared_mutex.h"

#include <pico/mutex.h>

void shared_mutex_init(shared_mutex_t* s_mtx)
{
  s_mtx->reader_count = 0;
  mutex_init(&s_mtx->reader_mutex);
  mutex_init(&s_mtx->writer_mutex);
}

void shared_mutex_enter_blocking_shared(shared_mutex_t* s_mtx)
{
  mutex_enter_blocking(&s_mtx->reader_mutex);
  s_mtx->reader_count++;
  if (s_mtx->reader_count == 1)
  {
    mutex_enter_blocking(&s_mtx->writer_mutex);
  }
  mutex_exit(&s_mtx->reader_mutex);
}

void shared_mutex_exit_shared(shared_mutex_t* s_mtx)
{
  mutex_enter_blocking(&s_mtx->reader_mutex);
  s_mtx->reader_count--;
  if (s_mtx->reader_count == 0)
  {
    mutex_exit(&s_mtx->writer_mutex);
  }
  mutex_exit(&s_mtx->reader_mutex);
}

void shared_mutex_enter_blocking_exclusive(shared_mutex_t* s_mtx)
{
  mutex_enter_blocking(&s_mtx->writer_mutex);
}

void shared_mutex_exit_exclusive(shared_mutex_t* s_mtx)
{
  mutex_exit(&s_mtx->writer_mutex);
}
