#define ECL_TIMER_TIMER_SEMAPHORE       uint32_t
#define ECL_TIMER_DISABLE_PROCESSING(x) __sync_fetch_and_add(&x, 1)
#define ECL_TIMER_ENABLE_PROCESSING(x)  __sync_fetch_and_sub(&x, 1)
#define ECL_TIMER_PROCESSING_ENABLED(x) (__sync_fetch_and_add(&x, 0) == 0)

