#ifndef TIME_ALIGN_HPP
#define TIME_ALIGN_HPP

#include "DeviceTime.hpp"
#include <deque>
#include <memory>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <iostream>
#include <string>
#include <mutex>

template<typename PkgTypeA, typename PkgTypeB, typename WallTimeType>
class TimeAlign {
public:
    using DeviceTimePtr = std::shared_ptr<DeviceTime>;

    explicit TimeAlign(
        const int data_fps,
        const rclcpp::Logger &logger = rclcpp::get_logger("Cam Publisher")
    ): logger(logger), initialized(false),
       Pf(1e2), Pc(0.0), Pt(1e12),
       Qf((1e2 / data_fps) * (1e2 / data_fps)), Qt((1e3 / data_fps) * (1e3 / data_fps)),
       data_time_gap_ns(1e9 / data_fps), wall_tol_sec(0.1), // Initial cov matrix： P = diag([1e3, 1e8])^2
       device_tol_ns(data_time_gap_ns / 5.0), // Process noise Q = diag([1e1, 1e3])^2 / (data_fps)^2
       obs_cov(1e12),
       ts_diff_ns(0.0) {
    }

    std::optional<std::pair<PkgTypeA, PkgTypeB> > addAPkg(DeviceTimePtr &&a_t, PkgTypeA &&a_pkg) {
        std::lock_guard lock(mutex_);

        if (!initialized) return {};
        auto result = addToQueue<DeviceTimePtr, PkgTypeA, PkgTypeB>(
            std::move(a_t), std::move(a_pkg), a_pkgs_deque, b_pkgs_deque,
            [this](const DeviceTimePtr &a, const DeviceTimePtr &b) { return matchDeviceA2B(*a, *b); }
        );
        if (result) {
            auto &&[Ta, Tb, Pa, Pb] = *result;
            update(*Ta, *Tb);
            return std::make_pair(std::move(Pa), std::move(Pb));
        }
        return {};
    }

    std::optional<std::pair<PkgTypeA, PkgTypeB> > addBPkg(DeviceTimePtr &&b_t, PkgTypeB b_pkg) {
        std::lock_guard lock(mutex_);

        if (!initialized) return {};
        auto result = addToQueue<DeviceTimePtr, PkgTypeB, PkgTypeA>(
            std::move(b_t), std::move(b_pkg), b_pkgs_deque, a_pkgs_deque,
            [this](const DeviceTimePtr &b, const DeviceTimePtr &a) { return matchDeviceB2A(*b, *a); }
        );
        if (result) {
            auto &&[Tb, Ta, Pb, Pa] = *result;
            update(*Ta, *Tb);
            return std::make_pair(std::move(Pa), std::move(Pb));
        }
        return {};
    }

    void addATms(WallTimeType a_wall, DeviceTimePtr &&a_t) {
        std::lock_guard lock(mutex_);

        if (auto result = addToQueue<WallTimeType, DeviceTimePtr, DeviceTimePtr >(
            std::move(a_wall), std::move(a_t), a_tms_deque, b_tms_deque,
            [this](const WallTimeType &maj, const WallTimeType &min) { return matchWall(maj, min); }
        )) {
            auto &&[Wa, Wb, Ta, Tb] = *result;
            initOrCheck(*Ta, *Tb);
        }
    }

    void addBTms(WallTimeType b_wall, DeviceTimePtr &&b_t) {
        std::lock_guard lock(mutex_);

        if (auto result = addToQueue<WallTimeType, DeviceTimePtr, DeviceTimePtr >(
            std::move(b_wall), std::move(b_t), b_tms_deque, a_tms_deque,
            [this](const WallTimeType &maj, const WallTimeType &min) { return matchWall(maj, min); }
        )) {
            auto &&[Wb, Wa, Tb, Ta] = *result;
            initOrCheck(*Ta, *Tb);
        }
    }

private:
    rclcpp::Logger logger;
    std::mutex mutex_;

    // Match queues
    template<typename Time_T, typename PKG_maj_T, typename PKG_min_T>
    std::optional<std::tuple<Time_T, Time_T, PKG_maj_T, PKG_min_T> > addToQueue(
        Time_T &&t_major,
        PKG_maj_T &&pkg_major,
        std::deque<std::pair<Time_T, PKG_maj_T> > &major_queue,
        std::deque<std::pair<Time_T, PKG_min_T> > &minor_queue,
        std::function<int(const Time_T &, const Time_T &)> matcher) {
        assert(major_queue.empty() or minor_queue.empty());
        while (!minor_queue.empty()) {
            const int matched = matcher(t_major, minor_queue.front().first);
            if (matched == 0) {
                // return: T_major, T_minor, PKG_major, PKG_minor
                auto res = std::make_tuple(
                    std::forward<Time_T>(t_major), std::move(minor_queue.front().first),
                    std::forward<PKG_maj_T>(pkg_major), std::move(minor_queue.front().second)
                );

                minor_queue.pop_front();
                return res;
            }

            // RCLCPP_WARN(logger, "Mismatched");

            if (matched > 0)
                minor_queue.pop_front();
            else
                return {};
        }

        major_queue.emplace_back(std::forward<Time_T>(t_major), std::forward<PKG_maj_T>(pkg_major));
        return {};
    }

    /*
    ((1 + fo) * ta + to) - tb ==  to + (ta - tb) + fo * ta
    | t_o + diff(t_a, t_b) + f_o * t_a.secFloat() | < device_tol
     */
    int matchDeviceA2B(const DeviceTime &t_a, const DeviceTime &t_b) const {
        const double time_diff = t_o + static_cast<double>(DeviceTime::diff(t_a, t_b)) + f_o * t_a.secFloat();
        return std::abs(time_diff) < device_tol_ns ? 0 : (time_diff < 0 ? -1 : 1);
    }

    int matchDeviceB2A(const DeviceTime &t_b, const DeviceTime &t_a) const {
        return -matchDeviceA2B(t_a, t_b);
    }

    int matchWall(const WallTimeType &t_a, const WallTimeType &t_b) const {
        return std::abs(t_a - t_b) < wall_tol_sec ? 0 : (t_a - t_b < 0 ? -1 : 1);
    }

    // Init Kalman filter or check the validity of Kalman filter
    void initOrCheck(const DeviceTime &a_device, const DeviceTime &b_device) {
        const int64_t device_diff = DeviceTime::diff(b_device, a_device);
        if (!initialized) {
            t_o = static_cast<double>(device_diff);
            f_o = 0.0;
            initialized = true;

            std::cout << "Init t_o: " << std::fixed << std::setprecision(9) << t_o << std::endl;
            RCLCPP_INFO(logger, "Init Kalman Filter");
        }
        else {
            if (matchDeviceA2B(a_device, b_device) != 0) {
                RCLCPP_ERROR(logger, "Error: f_o=%f, t_o=%f, device_diff=%ld", f_o, t_o, device_diff);
                throw std::runtime_error("Current Kalman filter is out of track!");
            }
        }
    }

    // Kalman update: t_b = (1 + f_o) * t_a + t_o
    void update(const DeviceTime &t_a, const DeviceTime &t_b) {
        Pf += Qf;
        Pt += Qt;

        const double ta_sec = t_a.secFloat();
        const double a_val = 2 * Pc + Pf * ta_sec;
        const double b_val = Pt + obs_cov + Pc * ta_sec;
        const double c_val = Pt + obs_cov + a_val * ta_sec;
        const double te = static_cast<double>(DeviceTime::diff(t_b, t_a));

        // Update fo and to
        const double new_f_o = ((b_val * f_o) + (a_val - Pc) * (te - t_o)) / c_val;
        const double new_t_o = t_o - (Pt + Pc * ta_sec) * (new_f_o * ta_sec - te + t_o) / c_val;

        // Update covariance
        const double new_Pf = (-Pc * Pc + Pf * (Pt + obs_cov)) / c_val;
        const double new_Pc = (-a_val * Pt + Pc * (b_val + Pt)) / c_val;
        const double new_Pt = (c_val * Pt - (b_val - obs_cov) * (b_val - obs_cov)) / c_val;

        f_o = new_f_o;
        t_o = new_t_o;
        Pf = new_Pf;
        Pc = new_Pc;
        Pt = new_Pt;

        ts_diff_ns = te;
    }

    // data deque
    std::deque<std::pair<DeviceTimePtr, PkgTypeA> > a_pkgs_deque;
    std::deque<std::pair<DeviceTimePtr, PkgTypeB> > b_pkgs_deque;

    // tms deque（第一元素为墙钟时间，第二元素为设备时间）
    std::deque<std::pair<WallTimeType, DeviceTimePtr > > a_tms_deque;
    std::deque<std::pair<WallTimeType, DeviceTimePtr > > b_tms_deque;

    // Kalman params
    bool initialized; // initialization indicator
    double f_o = 0; // freq shift
    double t_o = 0; // time shift
    double Pf, Pc, Pt; // State covariance
    double Qf, Qt; // Process noise

    double data_time_gap_ns;
    double wall_tol_sec; // wall time tolerance (unit: sec)
    double device_tol_ns;
    double obs_cov;

    double ts_diff_ns; // timestamps difference between camera and sync msgs
};

#endif // TIME_ALIGN_HPP
