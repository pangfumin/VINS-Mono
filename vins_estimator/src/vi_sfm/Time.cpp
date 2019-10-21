/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Modified and adapted by
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Original Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 *  Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *  Modified: Andreas Forster (an.forster@gmail.com)
 */

/**
 * @file Time.cpp
 * @brief Source file for the Time class.
 * @author Willow Garage Inc.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include "utility/Time.hpp"
#include <cmath>
#include <ctime>
#include <iomanip>
#include <stdexcept>
#include <limits>

/*********************************************************************
 ** Preprocessor
 *********************************************************************/

// Could probably do some better and more elaborate checking
// and definition here.
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)

/*********************************************************************
 ** Namespaces
 *********************************************************************/



/*********************************************************************
 ** Variables
 *********************************************************************/

const Duration DURATION_MAX(std::numeric_limits<int32_t>::max(), 999999999);
const Duration DURATION_MIN(std::numeric_limits<int32_t>::min(), 0);

const Time TIME_MAX(std::numeric_limits<uint32_t>::max(), 999999999);
const Time TIME_MIN(0, 1);

/*********************************************************************
** Cross Platform Functions
*********************************************************************/
/*
* These have only internal linkage to this translation unit.
* (i.e. not exposed to users of the time classes)
*/
void okvis_walltime(uint32_t& sec, uint32_t& nsec)  // NOLINT
#ifndef WIN32
    throw(NoHighPerformanceTimersException)
#endif
{
#if HAS_CLOCK_GETTIME
  timespec start;
  clock_gettime(CLOCK_REALTIME, &start);
  sec = start.tv_sec;
  nsec = start.tv_nsec;
#else
  struct timeval timeofday;
  gettimeofday(&timeofday, NULL);
  sec = timeofday.tv_sec;
  nsec = timeofday.tv_usec * 1000;
#endif
}
/**
* @brief Simple representation of the rt library nanosleep function.
*/
int okvis_nanosleep(const uint32_t& sec, const uint32_t& nsec) {
  timespec req = {sec, nsec};
  return nanosleep(&req, NULL);
}

/**
* @brief Go to the wall!
*
* @todo Fully implement the win32 parts, currently just like a regular sleep.
*/
bool okvis_wallsleep(uint32_t sec, uint32_t nsec) {
#if defined(WIN32)
  okvis_nanosleep(sec, nsec);
#else
  timespec req = {sec, nsec};
  timespec rem = {0, 0};
  while (nanosleep(&req, &rem)) {
    req = rem;
    }
#endif
  return true;
}

/*********************************************************************
 ** Class Methods
 *********************************************************************/

bool Time::useSystemTime() {
    return true;
}

bool Time::isSimTime() {
    return false;
}

bool Time::isSystemTime() {
    return !isSimTime();
}

Time Time::now() {
    Time t;
    okvis_walltime(t.sec, t.nsec);

    return t;
}

void Time::setNow(const Time&) {
    throw std::runtime_error("Unimplemented function");
}

void Time::init() {
}

void Time::shutdown() {
}

bool Time::isValid() {
    return true;
}

bool Time::waitForValid() {
    return waitForValid(WallDuration());
}

bool Time::waitForValid(const WallDuration& /*timeout*/) {
    /*okvis::WallTime start = ros::WallTime::now();
    while (!isValid() && !stopped_) {
      okvis::WallDuration(0.01).sleep();
      if (timeout > okvis::WallDuration(0, 0) && (okvis::WallTime::now() - start > timeout)) {
          return false;
      }
    }
    if (stopped_) {
      return false;
    }
    return true;*/
    throw std::runtime_error("Unimplemented function");
    return false;
}

std::ostream& operator<<(std::ostream& os, const Time &rhs) {
    os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    return os;
}

std::ostream& operator<<(std::ostream& os, const Duration& rhs) {
    os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    return os;
}

bool Time::sleepUntil(const Time& end) {
    if (Time::useSystemTime()) {
        Duration d(end - Time::now());
        if (d > Duration(0)) {
            return d.sleep();
        }

        return true;
    } else {
        Time start = Time::now();
        while ((Time::now() < end)) {
            okvis_nanosleep(0, 1000000);
            if (Time::now() < start) {
                return false;
            }
        }

        return true;
    }
}

bool WallTime::sleepUntil(const WallTime& end) {
    WallDuration d(end - WallTime::now());
    if (d > WallDuration(0)) {
        return d.sleep();
    }

    return true;
}

bool Duration::sleep() const {
    if (Time::useSystemTime()) {
        return okvis_wallsleep(sec, nsec);
    } else {
        Time start = Time::now();
        Time end = start + *this;
        if (start.isZero()) {
            end = TIME_MAX;
        }

        while ((Time::now() < end)) {
            okvis_wallsleep(0, 1000000);

            // If we started at time 0 wait for the
            // first actual time to arrive before starting the timer on
            // our sleep
            if (start.isZero()) {
                start = Time::now();
                end = start + *this;
            }

            // If time jumped backwards
            // from when we started sleeping, return immediately
            if (Time::now() < start) {
                return false;
            }
        }

        return true;
    }
}

std::ostream &operator<<(std::ostream& os, const WallTime &rhs) {
    os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    return os;
}

WallTime WallTime::now() {
    WallTime t;
    okvis_walltime(t.sec, t.nsec);

    return t;
}

std::ostream &operator<<(std::ostream& os, const WallDuration& rhs) {
    os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
    return os;
}

bool WallDuration::sleep() const {
    return okvis_wallsleep(sec, nsec);
}

void normalizeSecNSec(uint64_t& sec, uint64_t& nsec) {  // NOLINT
  uint64_t nsec_part = nsec % 1000000000UL;
  uint64_t sec_part = nsec / 1000000000UL;

  if (sec_part > UINT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec += sec_part;
  nsec = nsec_part;
}

void normalizeSecNSec(uint32_t& sec, uint32_t& nsec) {  // NOLINT
  uint64_t sec64 = sec;
  uint64_t nsec64 = nsec;

  normalizeSecNSec(sec64, nsec64);

  sec = (uint32_t)sec64;
  nsec = (uint32_t)nsec64;
}

void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec) {  // NOLINT
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part >= 1000000000L) {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0) {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < 0 || sec_part > INT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}

template class TimeBase<Time, Duration>;
template class TimeBase<WallTime, WallDuration>;
