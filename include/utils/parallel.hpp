#ifndef PARALLEL_HPP
#define PARALLEL_HPP

#ifndef USE_TBB
#define USE_TBB 1
#endif

#if USE_TBB
#define SINGLE_THREADED 0
#define MULTI_THREADED  1
#else
#define SINGLE_THREADED 1
#define MULTI_THREADED  0
#endif

#if USE_TBB
// ????
#define __TBB_PREVIEW_MUTEXES 1
#include <oneapi/tbb.h>
#include <oneapi/tbb/mutex.h>
#include <oneapi/tbb/rw_mutex.h>
typedef tbb::mutex mutex_t;
typedef tbb::rw_mutex rw_mutex_t;

#include <oneapi/tbb/parallel_for.h>
#else

struct lock_t {
    typedef scoped_lock lock_t;
};

typedef lock_t mutex_t;
typedef lock_t rw_mutex_t;

#endif

#endif