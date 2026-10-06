#ifndef ROBOTARM_CONTROLLERS_MALLOC_COUNTER_HPP
#define ROBOTARM_CONTROLLERS_MALLOC_COUNTER_HPP

// Counts heap allocations of the calling thread inside a scope.
//
// malloc_counter.cpp defines malloc, calloc, realloc, free and the aligned variants in the test
// executable. ELF symbol lookup prefers the executable, so every allocation of the process goes
// through them: the controller library, rclcpp, rcutils, ruckig, Eigen and operator new (which
// calls malloc in libstdc++). Unlike Eigen's EIGEN_RUNTIME_NO_MALLOC, nothing has to be
// recompiled, and allocations outside of Eigen are seen too.
//
// Only allocations of the thread that armed the counter are counted, so executor or logging threads
// running in the background do not disturb the measurement.
//
// Not usable together with ASan/TSan, which replace malloc themselves.

#include <cstddef>

namespace malloc_counter
{

// arm/disarm counting on the calling thread, nesting is not supported
void arm();
void disarm();

// number of allocations (malloc, calloc, realloc, aligned) since the last arm()
std::size_t count();

// call once at program start: the first backtrace() loads libgcc and allocates
void prime_backtrace();

// writes the stack of the first allocation since the last arm() to stderr, nothing if there was none
void print_first_allocation_backtrace();

struct ScopedArm
{
    ScopedArm() {arm();}
    ~ScopedArm() {disarm();}
    ScopedArm(const ScopedArm &) = delete;
    ScopedArm & operator=(const ScopedArm &) = delete;
};

// runs f with counting armed and returns the number of allocations it made
template<class F>
std::size_t count_allocations(F && f)
{
    {
        ScopedArm armed;
        f();
    }
    return count();
}

}  // namespace malloc_counter

#endif  // ROBOTARM_CONTROLLERS_MALLOC_COUNTER_HPP
