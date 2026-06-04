#ifndef CAMERA_CONFIG_HPP
#define CAMERA_CONFIG_HPP

#include "json.hpp"
#include <fstream>
#include <iostream>
#include <string>

using json = nlohmann::json;

class CameraConfig {
private:
    json config_;

public:
    // Camera parameters
    double exposure_time_us;
    int trigger_mode;
    int strobe_pulse_width;
    int buffer_count;
    int max_camera_enum;
    int image_capture_timeout_ms;
    int analog_gain;

    // Timing parameters
    int ticks_per_second;
    int ticks_per_frame;
    int ticks_per_tms;
    int fps;
    double frame_gap_ns;
    double tms_gap_ns;
    double frame_gap_tolerance_ns;
    double tms_gap_tolerance_ns;

    // Network parameters
    int ip_suffix;

    // Processing parameters
    int thread_count;

    // Constants
    double nanosecond_per_second;

    CameraConfig(const std::string& config_file = "src/camera_driver/src/configFiles/cam_ip50.json") {
        loadConfig(config_file);
        calculateDerivedValues();
    }

    bool loadConfig(const std::string& config_file) {
        try {
            std::ifstream file(config_file);
            if (!file.is_open()) {
                std::cerr << "Warning: Cannot open config file " << config_file
                         << ", using default values" << std::endl;
                setDefaultValues();
                return false;
            }

            file >> config_;
            file.close();

            // Load camera parameters
            exposure_time_us = config_["camera"]["exposure_time_us"];
            trigger_mode = config_["camera"]["trigger_mode"];
            strobe_pulse_width = config_["camera"]["strobe_pulse_width"];
            buffer_count = config_["camera"]["buffer_count"];
            max_camera_enum = config_["camera"]["max_camera_enum"];
            image_capture_timeout_ms = config_["camera"]["image_capture_timeout_ms"];
            analog_gain = config_["camera"]["analog_gain"];

            // Load timing parameters
            ticks_per_second = config_["timing"]["ticks_per_second"];
            ticks_per_frame = config_["timing"]["ticks_per_frame"];
            frame_gap_tolerance_ns = config_["timing"]["frame_gap_tolerance_ns"];
            tms_gap_tolerance_ns = config_["timing"]["tms_gap_tolerance_ns"];

            // Load network parameters
            ip_suffix = config_["network"]["ip_suffix"];

            // Load processing parameters
            thread_count = config_["processing"]["thread_count"];

            // Load constants
            nanosecond_per_second = config_["constants"]["nanosecond_per_second"];

            std::cout << "Configuration loaded successfully from " << config_file << std::endl;
            return true;

        } catch (const json::exception& e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
            setDefaultValues();
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Error loading config: " << e.what() << std::endl;
            setDefaultValues();
            return false;
        }
    }

    void setDefaultValues() {
        // Default camera parameters
        exposure_time_us = 1800.0;
        trigger_mode = 2;
        strobe_pulse_width = 500;
        buffer_count = 50;
        max_camera_enum = 2;
        image_capture_timeout_ms = 100;
        analog_gain = 800;

        // Default timing parameters
        ticks_per_second = 12000;
        ticks_per_frame = 499;
        frame_gap_tolerance_ns = 0.3e6;
        tms_gap_tolerance_ns = 0.2e6;

        // Default network parameters
        ip_suffix = 50;

        // Default processing parameters
        thread_count = 8;

        // Constants
        nanosecond_per_second = 1e9;
    }

    void calculateDerivedValues() {
        // Calculate derived timing values
        ticks_per_tms = ticks_per_second % ticks_per_frame + ticks_per_frame;
        fps = ticks_per_second / ticks_per_frame - 1;
        frame_gap_ns = nanosecond_per_second * ticks_per_frame / ticks_per_second;
        tms_gap_ns = nanosecond_per_second * ticks_per_tms / ticks_per_second;
    }

    void printConfig() const {
        std::cout << "=== Camera Configuration ===" << std::endl;
        std::cout << "Exposure Time: " << exposure_time_us << " us" << std::endl;
        std::cout << "Trigger Mode: " << trigger_mode << std::endl;
        std::cout << "Analog Gain: " << analog_gain << std::endl;
        std::cout << "FPS: " << fps << std::endl;
        std::cout << "Thread Count: " << thread_count << std::endl;
        std::cout << "IP Suffix: " << ip_suffix << std::endl;
        std::cout << "============================" << std::endl;
    }

    void updateAnalogGain(int new_gain) {
        analog_gain = new_gain;
        config_["camera"]["analog_gain"] = new_gain;
    }
};

#endif // CAMERA_CONFIG_HPP
