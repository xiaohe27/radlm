/*
 * Created on May, 2014
 * @author: Léonard Gérard leonard.gerard@sri.com
 *
 *
 * Flags handling
 *
 * Use an uint instead of a bitfield to be compatible.
 */

#pragma once

#include <stdint.h> //TODO use cstdint when c++11 is used

namespace radl {

typedef uint8_t flags_t;

// Functionnal flags are the first 4 bits
const flags_t FUNCTIONAL_FLAG = 15;
const flags_t STALE = 1;

// Failure flags are the others
const flags_t FAILURE_FLAGS = ~FUNCTIONAL_FLAG;
const flags_t TIMEOUT = 16;

// Handy user functions

// return if the stale flag of f is on
inline bool is_stale(flags_t f) { return f & STALE; };
// return if the timeout flag of f is on
inline bool is_timeout(flags_t f) { return f & TIMEOUT; };

// return true if any failure flag of f is on
inline bool is_failing(flags_t f) { return f & FAILURE_FLAGS; };

// Flags manipulation, ex: turn_on(STALE, x) turn on the stale flag of x
inline void turn_on(flags_t f, flags_t & x) { x |= f; };
inline void turn_off(flags_t f, flags_t & x) { x &= ~f; };

}
