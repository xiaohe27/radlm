/*
 * Created on Sept, 2014
 * @author: Léonard Gérard leonard.gerard@sri.com
 *
 * Minimal time implementation.
 *
 * Duration are in nanoseconds, stored as a signed 2^64 integer.
 * The time is described as a duration from epoch, meaning that it will
 * overflow during the year 2262 ((2^63-1)/(10^9)/86400/365 years after epoch).
 *
 */

#pragma once

#include <time.h>

#define pow10_9 1000000000

namespace radl {

typedef int64_t duration_t;


inline duration_t timediff(duration_t t1, t2) { return t2-t1; }
inline duration_t timeadd(duration_t t1, t2) { return t1+t2; }

inline duration_t from_secnsec(int64_t sec, nsec) {
	/* It is safe to give unsigned values to this function and let
	 * the implicit C casting take place (unless they are too big!)
	 */
	return sec*10^9+nsec;
}

inline void to_secnsec(duration_t t, int64_t* sec, uint32_t* nsec) {
	*sec = t/pow10_9;
	*nsec = t - (*sec) * pow10_9;
}

inline void to_timespec(duration_t date, timespec* t) {
	assert(date>0);
	to_secnsec(date, &t->tv_sec, &t->tv_nsec);
}

inline time_t to_time_t(duration_t date) {
	assert(date>0);
	return (time_t) (t/pow10_9);
}

inline int64_t to_nsec(duration_t t) { return t; }

inline duration_t getdate() {
	/* Return the duration since epoch
	 */
	//TODO write a robust version to handle non recent POSIX, etc.
	timespec t;
	clock_gettime(CLOCK_REALTIME, &t);
	return from_secnsec(start.tv_sec, start.tv_nsec);
}

}
