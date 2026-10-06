#include "malloc_counter.hpp"

#include <execinfo.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>

// glibc's implementations, the interposed functions below forward to them
extern "C" {
void * __libc_malloc(size_t size);
void * __libc_calloc(size_t n, size_t size);
void * __libc_realloc(void * ptr, size_t size);
void * __libc_memalign(size_t alignment, size_t size);
void __libc_free(void * ptr);
}

namespace
{

constexpr int max_frames = 64;

// trivially constructible thread_locals of the executable live in static TLS, accessing them does
// not allocate (which would recurse into malloc)
thread_local bool armed = false;
thread_local std::size_t allocations = 0;
thread_local void * first_frames[max_frames];
thread_local int first_frame_count = 0;

inline void record()
{
    if (!armed) {
        return;
    }
    if (allocations++ == 0) {
        // backtrace() is primed in main, but never count what it might allocate itself
        armed = false;
        first_frame_count = backtrace(first_frames, max_frames);
        armed = true;
    }
}

}  // namespace

extern "C" {

void * malloc(size_t size)
{
    record();
    return __libc_malloc(size);
}

void * calloc(size_t n, size_t size)
{
    record();
    return __libc_calloc(n, size);
}

void * realloc(void * ptr, size_t size)
{
    record();
    return __libc_realloc(ptr, size);
}

void * memalign(size_t alignment, size_t size)
{
    record();
    return __libc_memalign(alignment, size);
}

void * aligned_alloc(size_t alignment, size_t size)
{
    record();
    return __libc_memalign(alignment, size);
}

int posix_memalign(void ** out, size_t alignment, size_t size)
{
    record();
    if (alignment % sizeof(void *) != 0 || (alignment & (alignment - 1)) != 0) {
        return EINVAL;
    }
    void * p = __libc_memalign(alignment, size);
    if (!p && size != 0) {
        return ENOMEM;
    }
    *out = p;
    return 0;
}

void free(void * ptr)
{
    // freeing is not counted: it does not block like an allocation can, and every allocation in the
    // rt path is already caught on the malloc side
    __libc_free(ptr);
}

}  // extern "C"

namespace malloc_counter
{

void arm()
{
    allocations = 0;
    first_frame_count = 0;
    armed = true;
}

void disarm()
{
    armed = false;
}

std::size_t count()
{
    return allocations;
}

void prime_backtrace()
{
    void * frames[2];
    (void)backtrace(frames, 2);
}

void print_first_allocation_backtrace()
{
    if (first_frame_count == 0) {
        return;
    }
    fprintf(stderr, "first allocation of %zu, at:\n", allocations);
    fflush(stderr);
    backtrace_symbols_fd(first_frames, first_frame_count, STDERR_FILENO);
}

}  // namespace malloc_counter
