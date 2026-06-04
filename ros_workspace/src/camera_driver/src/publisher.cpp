// src/publisher.cpp

#include <chrono>
#include <memory>
#include <cstring>
#include <iostream>
#include <vector>
#include <atomic>
#include <thread>
#include <fstream>
#include <filesystem>
#include <queue>
#include <tuple>
#include <string>

#include <lz4.h>

#include "rcutils/logging.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/multi_array_layout__traits.hpp>
#include <std_msgs/msg/float64.hpp>

#include "my_msgs/msg/raw_img.hpp"
#include "my_msgs/msg/time_stamp.hpp"
#include "camera_driver/CameraApi.h"
#include "camera_driver/DeviceTime.hpp"
#include "camera_driver/TimeAlign.hpp"
#include "camera_driver/jsonConfig.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

typedef std::tuple<std::shared_ptr<my_msgs::msg::RawImg>, std::shared_ptr<CameraDeviceTime>, double> ImgPkg;

CameraConfig g_config;

// TODO: Solve the image loss caused by analog gain setting
std::atomic<int> analog_gain_atomic{800};

class Camera {
public:
    int camera_handle = -1;

    const double EXPOSURE_TIME_US;
    const int TRG_MODE;
    const int STROBE_PULSE_WIDTH;
    const int CAMERA_BUFFER_COUNT;
    const int MAX_CAMERA_ENUM;
    const int IMAGE_CAPTURE_TIMEOUT_MS;

    const int TICKS_PER_SECOND;
    const int TICKS_PER_FRAME;
    const int TICKS_PER_TMS;
    const int FPS;

    const double NANOSECOND_PER_SECOND;
    const double frame_gap_ns;
    const double tms_gap_ns;
    const double frame_gap_tol_ns;
    const double tms_gap_tol_ns;

    int curr_analog_gain;

    class CameraFrame {
        int camera_handle;

    public:
        BYTE *data = nullptr;
        tSdkFrameHead header{};

        explicit CameraFrame(const int camera_handle) {
            this->camera_handle = camera_handle;
        }

        ~CameraFrame() {
            if (data)
                CameraReleaseImageBuffer(camera_handle, data);
        }
    };

    explicit Camera(const rclcpp::Logger &logger = rclcpp::get_logger("Cam Publisher")):
        logger(logger),

        EXPOSURE_TIME_US(g_config.exposure_time_us),
        TRG_MODE(g_config.trigger_mode),
        STROBE_PULSE_WIDTH(g_config.strobe_pulse_width),
        CAMERA_BUFFER_COUNT(g_config.buffer_count),
        MAX_CAMERA_ENUM(g_config.max_camera_enum),
        IMAGE_CAPTURE_TIMEOUT_MS(g_config.image_capture_timeout_ms),
        TICKS_PER_SECOND(g_config.ticks_per_second),
        TICKS_PER_FRAME(g_config.ticks_per_frame),
        TICKS_PER_TMS(g_config.ticks_per_tms),
        FPS(g_config.fps),
        NANOSECOND_PER_SECOND(g_config.nanosecond_per_second),
        frame_gap_ns(g_config.frame_gap_ns),
        tms_gap_ns(g_config.tms_gap_ns),
        frame_gap_tol_ns(g_config.frame_gap_tolerance_ns),
        tms_gap_tol_ns(g_config.tms_gap_tolerance_ns),
        curr_analog_gain(g_config.analog_gain)

    {
        analog_gain_atomic = g_config.analog_gain;

        g_config.printConfig();

        // Validate parameters
        if (this->TRG_MODE > 0) {
            if (this->EXPOSURE_TIME_US * this->FPS >= 1e6)
                throw std::runtime_error("Exposure time too long for current FPS");
            if (this->STROBE_PULSE_WIDTH * this->FPS >= 1e6)
                throw std::runtime_error("Strobe pulse width too long for current FPS");
        }

        // SDK Initialization
        if (CameraSdkInit(1) != CAMERA_STATUS_SUCCESS)
            throw std::runtime_error("Camera SDK initialization failed");

        // Set system options
        if (CameraSetSysOption("NumBuffers", std::to_string(this->CAMERA_BUFFER_COUNT).c_str()) != CAMERA_STATUS_SUCCESS)
            throw std::runtime_error("Camera Set Option failed");

        // Enumerate devices
        tSdkCameraDevInfo camera_enum_list[this->MAX_CAMERA_ENUM];
        int camera_count = this->MAX_CAMERA_ENUM;
        if (CameraEnumerateDevice(camera_enum_list, &camera_count) != CAMERA_STATUS_SUCCESS) {
            throw std::runtime_error("Camera enumeration failed");
        }

        if (camera_count == 0)
            throw std::runtime_error("No camera found");

        // Initialize camera
        int camera_index = -1;
        RCLCPP_INFO(logger, "Found %d cameras", camera_count);

        std::string ip = "";
        for (int i = 0; i < camera_count; ++i) {
            ip = camera_enum_list[i].acPortType;
            size_t dash_pos = ip.find('-');
            std::string first_ip = ip.substr(0, dash_pos);

            size_t last_dot = first_ip.find_last_of('.');
            std::string last_part = first_ip.substr(last_dot + 1);

            int result = std::stoi(last_part);
            if (result == g_config.ip_suffix) {
                camera_index = i;
                break;
            }
        }

        if (camera_index == -1) {
            RCLCPP_ERROR(logger, "ERROR: Camera with ip %s not found!\n", ip.c_str());
            throw std::runtime_error("Target camera not found: " + ip);
        } else {
            RCLCPP_INFO(logger, "Found target camera at index %d with SN: %s\n", camera_index, ip.c_str());
        }

        if (CameraInit(&camera_enum_list[camera_index], -1, -1, &camera_handle) != CAMERA_STATUS_SUCCESS)
            throw std::runtime_error("Camera initialization failed");

        // Get camera capabilities
        tSdkCameraCapbility capability;
        CameraGetCapability(camera_handle, &capability);

        // Set image resolution
        CameraSetImageResolution(camera_handle, &capability.pImageSizeDesc[0]);

        // Set output format based on sensor type
        if (capability.sIspCapacity.bMonoSensor) {
            CameraSetIspOutFormat(camera_handle, CAMERA_MEDIA_TYPE_MONO8);
        }

        // Set gain settings
        CameraSetAnalogGain(camera_handle, curr_analog_gain);
        CameraSetGain(camera_handle, 69, 100, 197);

        // Set exposure settings
        CameraSetAeState(camera_handle, false);
        CameraSetExposureTime(camera_handle, this->EXPOSURE_TIME_US);

        // Set trigger mode
        CameraSetTriggerMode(camera_handle, this->TRG_MODE);

        // Set strobe parameters
        CameraSetStrobeMode(camera_handle, 2);
        CameraSetStrobePolarity(camera_handle, 0);
        CameraSetStrobeDelayTime(camera_handle, 0);
        CameraSetStrobePulseWidth(camera_handle, this->STROBE_PULSE_WIDTH);

        RCLCPP_INFO(logger, "Finished configuring camera.");

        if (CameraPlay(camera_handle) != CAMERA_STATUS_SUCCESS)
            throw std::runtime_error("Fail to start Camera Play");

        fill_buffer();
    }

    ~Camera() {
        if (camera_handle >= 0) {
            CameraStop(camera_handle);
            CameraUnInit(camera_handle);
            camera_handle = -1;
        }
        RCLCPP_INFO(logger, "Cleaning up camera resources");
    }

    ImgPkg get_frame() {
        fill_buffer();
        if (img_buffer.size() != img_buf_size + 1) { throw std::runtime_error("Img buffer is not full."); }

        auto data = img_buffer.top();
        img_buffer.pop();
        return data;
    }

private:
    rclcpp::Logger logger;
    bool is_ts_init = false;
    uint64_t init_cam_timestamp = 0;

    struct ImgTimeCompare {
        bool operator()(const ImgPkg &a, const ImgPkg &b) const {
            const auto &[img_a, ta, wa] = a;
            const auto &[img_b, tb, wb] = b;
            return *tb < *ta;
        }
    };

    std::priority_queue<ImgPkg, std::priority_queue<ImgPkg>::container_type, ImgTimeCompare> img_buffer;
    const int img_buf_size = 50;

    void fill_buffer() {
        while (img_buffer.size() <= img_buf_size) {
            CameraFrame buffer{camera_handle};
            if (const CameraSdkStatus status = CameraGetImageBuffer(camera_handle, &buffer.header, &buffer.data, this->IMAGE_CAPTURE_TIMEOUT_MS); status != CAMERA_STATUS_SUCCESS)
                throw std::runtime_error("Camera get frame fail code " + std::to_string(status));

            double wall_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

            auto imageMsg = std::make_shared<my_msgs::msg::RawImg>();
            imageMsg->height = buffer.header.iHeight;
            imageMsg->width = buffer.header.iWidth;
            imageMsg->data.resize(buffer.header.uBytes);
            std::memcpy(imageMsg->data.data(), buffer.data, buffer.header.uBytes);

            if (!is_ts_init)
                init_cam_timestamp = buffer.header.uiTimeStamp, is_ts_init = true;

            img_buffer.emplace(std::move(imageMsg), std::make_shared<CameraDeviceTime>(static_cast<uint64_t>(buffer.header.uiTimeStamp) - init_cam_timestamp), wall_time);

            if (curr_analog_gain != analog_gain_atomic) {
                CameraSetAnalogGain(camera_handle, analog_gain_atomic);
                curr_analog_gain = analog_gain_atomic;
            }
        }
    }
};


// TODO: is MUTEX necessary for manage time_aligner_ in CaptureFunc and SubCallback?
class CameraPubNode final : public rclcpp::Node {
public:
    Camera camera{this->get_logger()};

    CameraPubNode() : Node("camera_publish_node"),
                      time_aligner_(this->camera.FPS, this->get_logger()) {
        rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        this->declare_parameter<int>("analog_gain", analog_gain_atomic);
        int analog_gain = this->get_parameter("analog_gain").as_int();
        CameraSetAnalogGain(camera.camera_handle, analog_gain);
        RCLCPP_INFO(get_logger(), "Initial analog gain set to %d", analog_gain);
        parameter_callback_handle = this->add_on_set_parameters_callback(std::bind(&CameraPubNode::parametersCallback, this, std::placeholders::_1));

        for (size_t i = 0; i < thread_count; ++i) {
            threads_.emplace_back([this]() { this->consumerThread(); });
        }

        setupPubAndSub();
        startCapture();
    }

    ~CameraPubNode() override {
        RCLCPP_INFO(this->get_logger(), "Shutting down camera capture node");

        capture_finished_ = true;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        RCLCPP_INFO(get_logger(), "Stopping camera capture");

        stop_threads_.store(true);
        cv_.notify_all();

        for (auto &thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

private:
    std::thread capture_thread_;
    std::atomic<bool> capture_finished_{false};
    std::string image_save_path_ = "./";
    const int CAPIN_CH = 6;
    const size_t thread_count = g_config.thread_count;

    TimeAlign<std::shared_ptr<SyncDeviceTime>, std::shared_ptr<my_msgs::msg::RawImg>, double> time_aligner_;

    rclcpp::Publisher<my_msgs::msg::RawImg>::SharedPtr raw_pub_;
    rclcpp::Subscription<my_msgs::msg::TimeStamp>::SharedPtr ts_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fps_pub_;

    const double fps_update_rate = 1e-5;
    double avg_frame_gap = 1e9 / camera.FPS;

    std::vector<std::thread> threads_;
    std::queue<std::pair<my_msgs::msg::TimeStamp, my_msgs::msg::RawImg>> queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::atomic<bool> stop_threads_;

    void setupPubAndSub() {
        RCLCPP_INFO(get_logger(), "Set up publisher and subscriber");
        RCLCPP_INFO(get_logger(), "Subscribed to: /sync_data/capin/C%d", CAPIN_CH);

        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(800));
        custom_qos.reliable();
        custom_qos.durability_volatile();
        custom_qos.lifespan(rclcpp::Duration(1, 0));

        raw_pub_ = this->create_publisher<my_msgs::msg::RawImg>("/cam/raw_img", custom_qos);
        ts_sub_ = this->create_subscription<my_msgs::msg::TimeStamp>("/sync_data/capin/C" + std::to_string(CAPIN_CH), custom_qos,
                                                                     std::bind(&CameraPubNode::timestampCallback, this, std::placeholders::_1));
        fps_pub_ = this->create_publisher<std_msgs::msg::Float64>("/cam/fps", custom_qos);
    }

    void startCapture() {
        // Create directory if it doesn't exist
        if (!fs::exists(image_save_path_)) {
            try {
                fs::create_directories(image_save_path_);
                RCLCPP_INFO(get_logger(), "Created directory: %s", image_save_path_.c_str());
            } catch (const fs::filesystem_error &e) {
                RCLCPP_ERROR(get_logger(), "Failed to create directory: %s", e.what());
            }
        }

        RCLCPP_INFO(get_logger(), "Camera configured with: Exposure=%.3f ms, TriggerMode=%d, FPS=%d", this->camera.EXPOSURE_TIME_US / 1000.0, this->camera.TRG_MODE, this->camera.FPS);
        RCLCPP_INFO(get_logger(), "Starting camera capturing...");

        capture_thread_ = std::thread(&CameraPubNode::captureThreadFunc, this);
    }

    void captureThreadFunc() {
        RCLCPP_INFO(this->get_logger(), "[CaptureThread] started.");

        // const auto start_time = this->get_clock()->now();
        std::shared_ptr<CameraDeviceTime> prev_frame_time{};
        while (rclcpp::ok() && !capture_finished_) {
            // read out a frame from camera
            auto [img_msg, camera_device_time , wall_time] = camera.get_frame();

            // get frame gap
            if (!prev_frame_time) {
                prev_frame_time = camera_device_time;
                continue;
            }
            const int64_t frame_gap = *camera_device_time - *prev_frame_time;
            prev_frame_time = camera_device_time;

            // check the frame gap
            const bool is_tms = std::abs(frame_gap - camera.tms_gap_ns) < camera.tms_gap_tol_ns;
            const bool is_normal = is_tms || std::abs(frame_gap - camera.frame_gap_ns) < camera.frame_gap_tol_ns; // tms is also a normal frame

            if (is_tms) {
                auto camera_time = camera_device_time;
                time_aligner_.addBTms(wall_time, std::move(camera_time));
            }

            if (is_normal) {
                if (auto matched = time_aligner_.addBPkg(std::move(camera_device_time), std::move(img_msg))) {
                    auto [pkg_a, pkg_b] = *matched;
                    // publishFrame(pkg_a->convertToTsMsg(), *pkg_b);
                    enqueueFrame(pkg_a->convertToTsMsg(), *pkg_b);
                }

                avg_frame_gap = (1 - fps_update_rate) * avg_frame_gap + fps_update_rate * frame_gap;
            }
            else
                RCLCPP_INFO(get_logger(), "Unexpected frame gap: %.2f ms", static_cast<double>(frame_gap) / 1e6);
        }

        // const auto end_time = this->get_clock()->now();
        // const double duration_sec = (end_time - start_time).seconds();
    }

    void consumerThread() {
        while (true) {
            std::pair<my_msgs::msg::TimeStamp, my_msgs::msg::RawImg> data;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait(lock, [this]() { return !queue_.empty() || stop_threads_.load(); });

                if (stop_threads_.load() && queue_.empty()) {
                    return;
                }

                data = std::move(queue_.front());
                queue_.pop();
            }

            publishFrame(data.first, data.second);
        }
    }

    void publishFrame(const my_msgs::msg::TimeStamp &ts, my_msgs::msg::RawImg &imgMsg) const {
        imgMsg.ts = ts;

        const int original_size = static_cast<int>(imgMsg.data.size());
        const int max_compressed_size = LZ4_compressBound(original_size);

        std::vector<char> compressed_buffer(max_compressed_size);
        int compressed_size = LZ4_compress_default(
            reinterpret_cast<const char*>(imgMsg.data.data()),
            compressed_buffer.data(),
            original_size,
            max_compressed_size
        );

        if (compressed_size <= 0) {
            std::cerr << "LZ4 compression failed!" << std::endl;
            return;
        }

        compressed_buffer.resize(compressed_size);
        std::vector<uint8_t> compressed_data{
            std::make_move_iterator(compressed_buffer.begin()),
            std::make_move_iterator(compressed_buffer.end())
        };
        imgMsg.data.swap(compressed_data);
        raw_pub_->publish(imgMsg);

        std_msgs::msg::Float64 fps;
        fps.data = 1e9 / avg_frame_gap;
        fps_pub_->publish(fps);
    }

    void enqueueFrame(const my_msgs::msg::TimeStamp &ts, const my_msgs::msg::RawImg &imgMsg) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            queue_.emplace(ts, imgMsg);
        }
        cv_.notify_one(); // notify one waiting thread
    }

    // ReSharper disable once CppPassValueParameterByConstReference
    void timestampCallback(my_msgs::msg::TimeStamp::SharedPtr sync_msg) /* NOLINT(*-unnecessary-value-param) */ {
        auto sync_pkg = std::make_shared<SyncDeviceTime>(*sync_msg);

        if (sync_msg->tick == 1) { // is TMS
            const double wall_ts = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            auto sync_tms = sync_pkg;
            time_aligner_.addATms(wall_ts, std::move(sync_tms));
        }

        auto sync_time = sync_pkg;
        if (auto matched = time_aligner_.addAPkg(std::move(sync_time), std::move(sync_pkg))) {
            auto [pkg_a, pkg_b] = *matched;
            // publishFrame(pkg_a->convertToTsMsg(), *pkg_b);
            enqueueFrame(pkg_a->convertToTsMsg(), *pkg_b);
        }
    }

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle;

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param: parameters) {
            if (param.get_name() == "analog_gain") {
                analog_gain_atomic = param.as_int();
            }
        }
        return result;
    }
};


int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    try {
        const auto node = std::make_shared<CameraPubNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_capture"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
