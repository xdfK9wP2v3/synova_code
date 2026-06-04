
#ifndef DEVICETIME_H
#define DEVICETIME_H

#include <cstdint>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include "my_msgs/msg/time_stamp.hpp"


class DeviceTime {
public:
    static constexpr uint64_t NANOSECOND_PER_SEC = 1000000000ULL;

    virtual ~DeviceTime() = default;

    virtual uint64_t second() const = 0;

    virtual uint64_t nano_second() const = 0;

    double secFloat() const {
        return second() + nano_second() * 1e-9;
    }

    uint64_t total_nano_second() const {
        return second() * NANOSECOND_PER_SEC + nano_second();
    }

    // Note that a may be smaller than b
    static int64_t diff(const DeviceTime &a, const DeviceTime &b) {
        return static_cast<int64_t>(a.total_nano_second()) - static_cast<int64_t>(b.total_nano_second());
    }

    int64_t operator-(const DeviceTime &o) const {
        return diff(*this, o);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(7) << secFloat();
        return oss.str();
    }
};

class CameraDeviceTime final : public DeviceTime {
    uint64_t timestamp = 0;
    static constexpr uint64_t NANOSECOND_PER_CAMERA_TIMESTAMP = 100000ULL;

public:
    explicit CameraDeviceTime(const uint64_t &camera_timestamp): timestamp(camera_timestamp) {
    }

    uint64_t second() const override {
        return (timestamp * NANOSECOND_PER_CAMERA_TIMESTAMP) / NANOSECOND_PER_SEC;
    }

    uint64_t nano_second() const override {
        return (timestamp * NANOSECOND_PER_CAMERA_TIMESTAMP) % NANOSECOND_PER_SEC;
    }

    bool operator <(const CameraDeviceTime &other) const { return this->timestamp < other.timestamp; }
};

class SyncDeviceTime final : public DeviceTime {
public:
    SyncDeviceTime()
        : power_second(0), utc_second(0),
          tick(0), ticks_per_second(0),
          subtick(0), subtick_per_tick(0), clock(0), clocks_per_second(0) {
    }

    explicit SyncDeviceTime(const my_msgs::msg::TimeStamp &ts_msg)
        : power_second(ts_msg.pwr_sec), utc_second(ts_msg.utc_sec),
          tick(ts_msg.tick), ticks_per_second(ts_msg.tick_per_sec),
          subtick(ts_msg.subtick), subtick_per_tick(ts_msg.subtick_per_tick), clock(ts_msg.clk), clocks_per_second(ts_msg.clk_per_sec) {
        if (ticks_per_second == 0)
            throw std::invalid_argument("ticks_per_sec cannot be zero.");

        if (subtick_per_tick == 0)
            throw std::invalid_argument("subtick_per_tick cannot be zero.");
    }

    uint64_t second() const override { return power_second + (rolled_nanosecond() / NANOSECOND_PER_SEC); }
    uint64_t nano_second() const override { return rolled_nanosecond() % NANOSECOND_PER_SEC; }

    // convert to TimeStamp
    my_msgs::msg::TimeStamp convertToTsMsg() const {
        my_msgs::msg::TimeStamp ts_msg;
        ts_msg.pwr_sec = power_second;
        ts_msg.utc_sec = utc_second;
        ts_msg.tick = tick;
        ts_msg.tick_per_sec = ticks_per_second;
        ts_msg.subtick = subtick;
        ts_msg.subtick_per_tick = subtick_per_tick;
        ts_msg.clk = clock;
        ts_msg.clk_per_sec = clocks_per_second;
        return ts_msg;
    }

private:
    uint64_t power_second;
    uint64_t utc_second;
    uint64_t tick;
    uint64_t ticks_per_second;
    uint64_t subtick;
    uint64_t subtick_per_tick;
    uint64_t clock;
    uint64_t clocks_per_second;

    uint64_t rolled_nanosecond() const {
        const uint64_t tick_term = (tick * NANOSECOND_PER_SEC) / ticks_per_second; // _tick / _ticks_per_sec <= 1
        const uint64_t subtick_term = (subtick * NANOSECOND_PER_SEC) / (ticks_per_second * subtick_per_tick); // _subtick / _subtick_per_tick <= 1
        return tick_term + subtick_term;
    }
};

#endif //DEVICETIME_H
