/*
 * Author: BlindZhou
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <array>
#include "simple_log.h"
#include <math.h>
#include <sstream>

typedef std::array<float, 3> Vectorf3;

namespace common {
    inline float radian_to_degree(const float &radian) {
        float degree = radian * 180.0 * M_1_PI;
        return degree;
    }

    inline float degree_to_radian(const float &degree) {
        float radian = degree * M_PI / 180;
        return radian;
    }

    inline double normalize_angle_positive(double angle) {
        return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
    }

    inline double normalize_angle(double angle) {
        double a = normalize_angle_positive(angle);
        if (a > M_PI) {
            a -= 2.0 * M_PI;
        }
        return a;
    }
}

class Clock {
public:
    Clock() {
        set_to_now();
    }

    Clock(const int64_t stamp_us) {
        m_stamp = stamp_us;
    }

    inline void set_to(int64_t stamp_us) {
        m_stamp = stamp_us;
    }

    inline void set_to_zero() {
        m_stamp = 0;
    }

    inline void set_to_now() {
        m_stamp = time_stamp_us();
    }

    inline double to_now_s() {
        return (double) 1e-6 * (time_stamp_us() - m_stamp);
    }

    static double to_now_s(const int64_t stamp) {
        return (double) 1e-6 * (time_stamp_us() - stamp);
    }

    inline double to_now_ms() {
        return (double) 1e-3 * (time_stamp_us() - m_stamp);
    }

    static double to_now_ms(const int64_t stamp_us) {
        return (double) 1e-3 * (time_stamp_us() - stamp_us);
    }

    inline double to_now_us() {
        return (double) (time_stamp_us() - m_stamp);
    }

    static double to_now_us(const int64_t stamp_us) {
        return (double) (time_stamp_us() - stamp_us);
    }

    static int64_t time_stamp_us() {
        std::chrono::steady_clock::time_point tp = std::chrono::steady_clock::now();
        std::chrono::microseconds us =
                std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
        return (int64_t) us.count();
    }

    static double time_stamp() {
        return 1e-6 * time_stamp_us();
    }

    operator double() {
        return 1e-6 * time_stamp_us();
    }

    operator float() {
        return float(1e-6 * time_stamp_us());
    }

    operator int64_t() {
        return time_stamp_us();
    }

    static std::string date() {
        std::time_t now = std::time(nullptr);
        static char timeBuffer[256];
        std::strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        const auto now_time_point = std::chrono::system_clock::now();

        std::stringstream ss;
        ss <<timeBuffer << "." << std::setw(3) << std::setfill('0') <<
             std::chrono::duration_cast<std::chrono::milliseconds>
             (now_time_point.time_since_epoch()).count() % 1000 ;
        return ss.str();
    }
private:
    double m_stamp;
};

#define DO_EVERY_SEC(CLOCK, TIME, FUNC) \
    do                                  \
    {                                   \
        if (CLOCK.to_now_s() > TIME)    \
        {                               \
            FUNC;                       \
            CLOCK.set_to_now();         \
        }                               \
    } while (0);

