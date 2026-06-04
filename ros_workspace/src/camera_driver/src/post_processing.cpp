
#include "camera_driver/CameraApi.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cstring>
#include <cassert>

#define BIN_SAVE_PATH "~/Downloads/camera_data/raw_bin" // TODO: share with subsciber.cpp
#define IMAGE_SAVE_PATH "~/Downloads/camera_data/images"

static int g_hCamera = -1;
static const int TOTAL_FRAME_NUM = 100000;  // TODO: Share with publisher.cpp
const int g_numThreads = 10;

static std::queue<std::vector<uint8_t>> g_postQueue;
static std::mutex g_postMutex;
static std::condition_variable g_postCV;
static std::atomic<bool> g_isReadingFinished(false);

static std::mutex g_tsMutex;
static std::vector<unsigned int> g_timeStampVec;

// TODO: share with publisher.cpp, should be in a common header file
struct FrameData
{
    int frameIdx;
    tSdkFrameHead frameInfo;
    std::vector<BYTE> buffer;
};

bool ParseFrameData(const std::vector<uint8_t> &rawBytes, FrameData &outFrame)
{
    size_t offset = 0;

    // 1) frameIdx
    if (offset + sizeof(int) > rawBytes.size())
        return false;
    std::memcpy(&outFrame.frameIdx, rawBytes.data() + offset, sizeof(int));
    offset += sizeof(int);

    // 2) tSdkFrameHead
    if (offset + sizeof(tSdkFrameHead) > rawBytes.size())
        return false;
    std::memcpy(&outFrame.frameInfo, rawBytes.data() + offset, sizeof(tSdkFrameHead));
    offset += sizeof(tSdkFrameHead);

    // 3) buffer
    size_t bufferSize = rawBytes.size() - offset;
    outFrame.buffer.resize(bufferSize);
    if (bufferSize > 0)
        std::memcpy(outFrame.buffer.data(), rawBytes.data() + offset, bufferSize);

    return true;
}

void workerFunc(int threadId, int numThreads)
{
    const int frameSize = 1600 * 1200 * 3;
    std::vector<BYTE> localRgbBuffer(frameSize, 0);

    while (true)
    {
        std::vector<uint8_t> rawBytes;
        {
            std::unique_lock<std::mutex> lk(g_postMutex);
            g_postCV.wait(lk, []{
                return !g_postQueue.empty() || g_isReadingFinished.load();
            });

            if (g_postQueue.empty()) {
                if (g_isReadingFinished.load()) {
                    break;
                } else {
                    continue;
                }
            }

            rawBytes = std::move(g_postQueue.front());
            g_postQueue.pop();
        }

        FrameData frameData;
        bool ok = ParseFrameData(rawBytes, frameData);
        if (!ok) {
            std::cerr << "[Worker " << threadId << "] ParseFrameData failed" << std::endl;
            continue;
        }

        CameraSdkStatus procStatus = CameraImageProcess(
            g_hCamera,
            frameData.buffer.data(),
            localRgbBuffer.data(),
            &frameData.frameInfo
        );
        if (procStatus != CAMERA_STATUS_SUCCESS) {
            std::cerr << "[Worker " << threadId << "] CameraImageProcess error on frameIdx = "
                      << frameData.frameIdx << std::endl;
            continue;
        }

        char filename[256];
        sprintf(filename, "%s/%06d.bmp", IMAGE_SAVE_PATH, frameData.frameIdx);
        CameraSdkStatus saveStatus = CameraSaveImage(
            g_hCamera,
            filename,
            localRgbBuffer.data(),
            &frameData.frameInfo,
            FILE_BMP,
            0
        );
        if (saveStatus != CAMERA_STATUS_SUCCESS) {
            std::cerr << "[Worker " << threadId << "] CameraSaveImage error on frameIdx = "
                      << frameData.frameIdx << std::endl;
        }

        {
            std::lock_guard<std::mutex> lk(g_tsMutex);
            if (frameData.frameIdx >= 0 && frameData.frameIdx < (int)g_timeStampVec.size()) {
                g_timeStampVec[frameData.frameIdx] = frameData.frameInfo.uiTimeStamp;
            }
        }
    }

    std::cout << "[Worker " << threadId << "] exited." << std::endl;
}

int main()
{
    // TODO: Check if SDK can be used in offline mode
    CameraSdkInit(1);

    tSdkCameraDevInfo tCameraEnumList[4];   // TODO: Check the meaning of this array size
    int camCnt = 1;
    CameraEnumerateDevice(tCameraEnumList, &camCnt);
    if (camCnt < 1) {
        std::cerr << "No camera found. Offline processing failed." << std::endl;
    } else {
        int status = CameraInit(&tCameraEnumList[0], -1, -1, &g_hCamera);
        if (status != CAMERA_STATUS_SUCCESS) {
            std::cerr << "CameraInit failed. code=" << status << std::endl;
            return -1;
        }

        CameraPlay(g_hCamera);   // TODO: check if this is necessary
    }

    g_timeStampVec.resize(TOTAL_FRAME_NUM, 0);

    std::vector<std::thread> workers;
    workers.reserve(g_numThreads);
    for (int i = 0; i < g_numThreads; ++i) {
        workers.emplace_back(workerFunc, i, g_numThreads);
    }

    char binFilePath[256];
    sprintf(binFilePath, "%s/frames.bin", BIN_SAVE_PATH);
    std::ifstream ifs(binFilePath, std::ios::binary);
    if (!ifs.is_open()) {
        std::cerr << "Cannot open " << binFilePath << std::endl;
        g_isReadingFinished = true;
        g_postCV.notify_all();

        for (auto &t : workers) {
            if (t.joinable()) {
                t.join();
            }
        }
        if (g_hCamera >= 0) {
            CameraStop(g_hCamera);
            CameraUnInit(g_hCamera);
        }
        return -1;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    int readFrameCount = 0;

    while (true) {
        if (!ifs.good()) break;

        int frameIdx;
        tSdkFrameHead frameInfo;

        if (!ifs.read(reinterpret_cast<char*>(&frameIdx), sizeof(frameIdx))) {
            break;
        }
        if (!ifs.read(reinterpret_cast<char*>(&frameInfo), sizeof(frameInfo))) {
            break;
        }

        // Read buffer
        std::vector<uint8_t> buffer(frameInfo.uBytes);
        if (!buffer.empty()) {
            if (!ifs.read(reinterpret_cast<char*>(buffer.data()), buffer.size())) {
                break;
            }
        }

        const size_t totalSize = sizeof(int) + sizeof(tSdkFrameHead) + buffer.size();
        std::vector<uint8_t> rawBytes(totalSize);

        size_t offset = 0;
        std::memcpy(rawBytes.data() + offset, &frameIdx, sizeof(frameIdx));
        offset += sizeof(frameIdx);

        std::memcpy(rawBytes.data() + offset, &frameInfo, sizeof(frameInfo));
        offset += sizeof(frameInfo);

        if (!buffer.empty()) {
            std::memcpy(rawBytes.data() + offset, buffer.data(), buffer.size());
        }

        {
            std::lock_guard<std::mutex> lk(g_postMutex);
            g_postQueue.push(std::move(rawBytes));
        }
        g_postCV.notify_one();

        readFrameCount++;
    }
    ifs.close();

    g_isReadingFinished = true;
    g_postCV.notify_all();

    for (auto &t : workers) {
        if (t.joinable()) {
            t.join();
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    double durationSec = std::chrono::duration<double>(endTime - startTime).count();
    std::cout << "[PostProcessing] Processed " << readFrameCount << " frames in "
              << durationSec << " sec." << std::endl;

    {
        char csvFilename[256];
        sprintf(csvFilename, "%s/timestamp.csv", IMAGE_SAVE_PATH);
        std::ofstream ofs(csvFilename);  // TODO: set the path
        if (!ofs.is_open()) {
            std::cerr << "Cannot open timestamp.csv for writing" << std::endl;
        } else {
            for (int i = 0; i < readFrameCount && i < (int)g_timeStampVec.size(); ++i) {
                ofs << g_timeStampVec[i] << "\n";
            }
            ofs.close();
            std::cout << "[PostProcessing] timestamp.csv saved." << std::endl;
        }
    }

    if (g_hCamera >= 0) {
        CameraStop(g_hCamera);
        CameraUnInit(g_hCamera);
    }

    std::cout << "[PostProcessing] Finished all tasks." << std::endl;
    return 0;
}
