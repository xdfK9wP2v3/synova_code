#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <cstring>
#include <filesystem>

#include "camera_driver/CameraApi.h"

#define BIN_SAVE_PATH "~/Downloads/camera_data/raw_bin"
#define IMAGE_SAVE_PATH "~/Downloads/camera_data/images"

static std::vector<unsigned int> g_timeStampVec;

class CameraBinSubscriber : public rclcpp::Node
{
public:
    CameraBinSubscriber()
    : Node("camera_bin_subscriber"),
      frames_saved_(0)
    {
        // configure subscriber params
        auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(200));
        custom_qos.reliable();
        custom_qos.durability_volatile();

        // TODO: Check the queue size
        bin_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "camera_bin", custom_qos, std::bind(&CameraBinSubscriber::binCallback, this, std::placeholders::_1)
        );

        char binFilePath[256];
        sprintf(binFilePath, "%s/frames.bin", BIN_SAVE_PATH);
        bin_ofs_.open(binFilePath, std::ios::binary | std::ios::out);
        if (!bin_ofs_.is_open()) {
            RCLCPP_ERROR(get_logger(), "Cannot open frames.bin for writing!");
            throw std::runtime_error("Failed to open frames.bin");
        }

        RCLCPP_INFO(get_logger(), "CameraBinSubscriber: frames.bin has been opened for writing.");
    }

    ~CameraBinSubscriber()
    {
        // save timestamps
        char csvFilename[256];
        sprintf(csvFilename, "%s/sub_timestamp.csv", IMAGE_SAVE_PATH);
        std::ofstream ofs(csvFilename);  // TODO: set the path
        if (!ofs.is_open()) {
            std::cerr << "Cannot open timestamp.csv for writing" << std::endl;
        } else {
            for (int i = 0; i < (int)g_timeStampVec.size(); ++i) {
                ofs << g_timeStampVec[i] << "\n";
            }
            ofs.close();
            std::cout << "[CaptureThread] timestamp.csv saved." << std::endl;
        }

        if (bin_ofs_.is_open()) {
            bin_ofs_.close();
            RCLCPP_INFO(get_logger(), "CameraBinSubscriber: frames.bin has been closed.");
        }
    }

private:
    /**
     * @brief
     */
    void binCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        size_t offset = 0;

        int frameIdx = 0;
        std::memcpy(&frameIdx, msg->data.data() + offset, sizeof(frameIdx));
        offset += sizeof(frameIdx);

        tSdkFrameHead frameInfo;
        std::memcpy(&frameInfo, msg->data.data() + offset, sizeof(frameInfo));
        offset += sizeof(frameInfo);

        g_timeStampVec.push_back(frameInfo.uiTimeStamp);

        // size_t bufferSize = msg->data.size() - offset;
        //
        // bin_ofs_.write(reinterpret_cast<const char*>(&frameIdx), sizeof(frameIdx));
        // bin_ofs_.write(reinterpret_cast<const char*>(&frameInfo), sizeof(frameInfo));
        // bin_ofs_.write(reinterpret_cast<const char*>(msg->data.data() + offset), bufferSize);
        current_frame_idx_ = frameIdx;
        frames_saved_++;

        if (frames_saved_ % 1000 == 0)
            RCLCPP_INFO(this->get_logger(), "Received frameIdx = %d, total saved = %d", current_frame_idx_ + 1, frames_saved_);
    }

    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr bin_sub_;
    std::ofstream bin_ofs_;
    int frames_saved_;
    int current_frame_idx_;
};

void ClearDirectory(const std::string& dirPath) {
    try {
        if (!std::filesystem::exists(dirPath) || !std::filesystem::is_directory(dirPath)) {
            std::cerr << "Directory does not exist: " << dirPath << std::endl;
            return;
        }

        for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
            if (std::filesystem::is_regular_file(entry.status())) {
                std::filesystem::remove(entry.path());
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

int main(int argc, char* argv[])
{
    ClearDirectory(BIN_SAVE_PATH);
    ClearDirectory(IMAGE_SAVE_PATH);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraBinSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
