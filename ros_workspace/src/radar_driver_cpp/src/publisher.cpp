#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <iostream>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "radar_udp.h"
#include "ftdi_comm.h"
#include "awr2243_config_parser.h"
#include "my_msgs/msg/raw_radar.hpp"
#include "my_msgs/msg/time_stamp.hpp"

using namespace std::chrono_literals;

class RadarPubNode final : public rclcpp::Node {
public:
    RadarPubNode() : Node("RadarPublisher"), radar_start_flag_(false), frame_count_(0) {
        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(600));
        custom_qos.reliable();
        custom_qos.durability_volatile();
        custom_qos.lifespan(rclcpp::Duration(1, 0));

        data_publisher_ = this->create_publisher<my_msgs::msg::RawRadar>("/radar/sensor_data", custom_qos);
        trout_subscriber_ = this->create_subscription<my_msgs::msg::TimeStamp>(
            "/sync_data/trout/C" + std::to_string(TROUT_CH), custom_qos,
            std::bind(&RadarPubNode::trout_callback, this, std::placeholders::_1));

        this->declare_parameter<std::string>("awr2243_config_select", "max30");
        this->get_parameter("awr2243_config_select", awr2243_config_select_);
        RCLCPP_INFO(this->get_logger(), "Config select: %s", awr2243_config_select_.c_str());

//        const char* home = std::getenv("HOME");
        config_path = "src/radar_driver_cpp/src/configFiles/AWR2243_mmwaveconfig_" + awr2243_config_select_ + ".txt";
        dca_config_path = "src/radar_driver_cpp/src/configFiles/dca_config.txt";
        adc_params = awr2243_read_config(config_path);

        std::string subnet = "11";
        if(const char* env_subnet = std::getenv("SUBNET")) {
            subnet = env_subnet;
        }
        dca_ = std::make_shared<RadarUDP>("192.168." + subnet + ".30", "192.168." + subnet + ".99", 4096, 4098);

        bytes_per_frame_ = adc_params.chirps * adc_params.rx * adc_params.tx * adc_params.IQ * adc_params.samples *
                           adc_params.bytes;
        pkt_per_frame_ = (bytes_per_frame_ + data_bytes_per_pkt_ - 1) / data_bytes_per_pkt_;

        radar_thread_ = std::thread(&RadarPubNode::radar_capture_thread, this);
    }

    ~RadarPubNode() override {
        radar_start_flag_ = false;
        if (radar_thread_.joinable()) {
            radar_thread_.join();
        }
        AWR2243_sensorStop();
        AWR2243_waitSensorStop();
        if (dca_) {
            dca_->stream_stop();
        }
        RCLCPP_INFO(this->get_logger(), "Radar publisher closed.");
    }

private:
    const int TROUT_CH = 7;

    void trout_callback(const my_msgs::msg::TimeStamp &msg) {
        if (radar_start_flag_) {
            current_ts = msg;
        }
    }

    void radar_capture_thread() {
        try {
            AWR2243_sensorStop();
            AWR2243_waitSensorStop();
            dca_->reset_radar();
            dca_->reset_fpga();

            AWR2243_init(config_path);
            AWR2243_setFrameCfg(0);
            dca_->config_fpga(dca_config_path);

            uint32_t packet_cnt = 0;
            uint32_t firstSeqNum = 0, lastSeqNum = 0;
            uint32_t no_data_cnt = 5;
            uint32_t seq_shift = 0;

            const auto queue_size = dca_->udp_read_thread_init(2 * pkt_per_frame_);
            dca_->stream_start();
            radar_start_flag_ = true;
            AWR2243_sensorStart();
            RCLCPP_INFO(this->get_logger(), "radar is started");

            while (radar_start_flag_) {
                const auto data = RadarUDP::udp_read_thread_get_packets(pkt_per_fetch_, udp_timeout_ms, true, firstSeqNum, lastSeqNum);
                if (!firstSeqNum) {
                    RCLCPP_WARN(this->get_logger(), "no udp data from DCA");
                    no_data_cnt--;
                    if (!no_data_cnt){
                        throw std::runtime_error("always no data from DCA, check USB connection");
                    }
                    continue;
                }
                if ((firstSeqNum - 1 - seq_shift) % pkt_per_fetch_ != 0 ||  (lastSeqNum - seq_shift) % pkt_per_fetch_ != 0) {
                    RCLCPP_ERROR(this->get_logger(), "Packet loss happened:  %u - %u", firstSeqNum, lastSeqNum);
                    seq_shift = (firstSeqNum - 1) % pkt_per_fetch_;
                    throw std::runtime_error("Packet loss happened, restart");
                }
                packet_cnt += pkt_per_fetch_;

                if (packet_cnt >= pkt_per_frame_) {
                    packet_cnt -= static_cast<int>(pkt_per_frame_);
                    frame_count_++;
                    RCLCPP_INFO(this->get_logger(), "[%s] Captured %u frames, pwr_sec %u, utc_sec %u",
                                awr2243_config_select_.c_str(),
                                frame_count_,
                                current_ts.pwr_sec,
                                current_ts.utc_sec);
                }
                auto radar_msg = my_msgs::msg::RawRadar();
                radar_msg.data = data;
                if (adc_params.triggerSelect == "hardware")
                    radar_msg.ts = current_ts;
                data_publisher_->publish(radar_msg);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Radar capture thread error: %s", e.what());
            AWR2243_sensorStop();
            AWR2243_waitSensorStop();
//            if (dca_) {
//                dca_->stream_stop();
//            }
            throw e;
        }
    }

    rclcpp::Publisher<my_msgs::msg::RawRadar>::SharedPtr data_publisher_;
    rclcpp::Subscription<my_msgs::msg::TimeStamp>::SharedPtr trout_subscriber_;

    std::shared_ptr<RadarUDP> dca_;
    std::string config_path;
    ADC_PARAMS adc_params{};

    std::string dca_config_path;

    size_t bytes_per_frame_{};
    size_t pkt_per_frame_;
    const int data_bytes_per_pkt_ = 1456;
    const int pkt_per_fetch_ = 500;
    const int udp_timeout_ms = 1500;

    my_msgs::msg::TimeStamp current_ts;

    std::thread radar_thread_;
    std::atomic<bool> radar_start_flag_;
    uint32_t frame_count_;
    std::string awr2243_config_select_;
};

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<RadarPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
