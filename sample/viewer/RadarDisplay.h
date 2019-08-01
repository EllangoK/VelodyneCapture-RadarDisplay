#include <vector>
#include <queue>
#include <string.h>
#include <inttypes.h>
#include <cmath>
#include <algorithm>

#include <stdlib.h>
#include <thread>
#include <functional>
#include <stdexcept>
#include <chrono>
#include <ctime>
#include <atomic>
#include <mutex>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <opencv2/opencv.hpp>

namespace radar {

class RadarDisplay {
protected:
    std::thread* threadQueueRead = nullptr;
    std::thread* threadQueueWrite = nullptr;
    std::atomic_bool run = { false };
    std::atomic_bool newData = { false };
    std::mutex mutex;
    std::queue<std::vector<cv::Vec3f> > queue;
    std::atomic_int queueSize = { 0 };

    int sockfd, newsockfd, portno;
    char msgBuffer[29];
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;

    float offsetX, offsetY, offsetZ, lidarOffsetZ = 0;
    float avgCurbHeight = 0, groundZ = 0, scaleZ = 1, scaleX = 1;
    int cycles = 0;
    std::atomic_bool queueBuildFlag = { false };
    std::vector<cv::Vec3f> lidarPointsInRange, firstObject, secondObject;
    std::string radarData;

public:
    RadarDisplay(int port, float X, float Y, float Z)
    {
        portno = port;
        offsetX = X;
        offsetY = Y;
        offsetZ = Z;
        startServer();
        threadRadarRead();
    };
    ~RadarDisplay() { close(); };

    bool isQueueBuildOver() { return queueBuildFlag; }
    void resetQueueBuild() { queueBuildFlag = { false }; }
    bool isRun()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return (run || !queue.empty());
    }
    bool isInPercentTolerance(float measured, float actual,
        float percentTolerance)
    {
        return (fabs((measured - actual) / actual) * 100. <= percentTolerance);
    }
    bool isEmpty()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return queue.empty();
    }
    size_t getQueueSize() { return queueSize; }
    template <typename T1, typename T2>
    typename T1::value_type quant(const T1& x, T2 q)
    {
        assert(q >= 0.0 && q <= 1.0);

        const auto n = x.size();
        const auto id = (n - 1) * q;
        const auto lo = floor(id);
        const auto hi = ceil(id);
        const auto qs = x[lo];
        const auto h = (id - lo);

        return (1.0 - h) * qs + h * x[hi];
    }

    void startServer()
    {
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            error("Opening Socket Error");
        }
        bzero((char*)&serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(portno);
        if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            error("Binding Error");
        }
        listen(sockfd, 4);
        clilen = sizeof(cli_addr);
        newsockfd = accept(sockfd, (struct sockaddr*)&cli_addr, &clilen);
        if (newsockfd < 0) {
            error("Connection Accept Error");
        }
        bzero(msgBuffer, 29);
    }
    void threadRadarRead()
    {
        run = true;
        auto funcUpdateQueue = std::bind(&RadarDisplay::updateQueue, this);
        unsigned int updateTime = 50;
        threadQueueWrite = new std::thread([funcUpdateQueue, updateTime]() {
            while (updateTime) {
                auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(updateTime);
                funcUpdateQueue();
                std::this_thread::sleep_until(x);
            }
        });
        auto funcExposeQueue = std::bind(&RadarDisplay::exposeQueue, this);
        unsigned int exposeTime = 250;
        threadQueueRead = new std::thread([funcExposeQueue, exposeTime]() {
            while (exposeTime) {
                auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(exposeTime);
                funcExposeQueue();
                std::this_thread::sleep_until(x);
            }
        });
    }
    void updateQueue()
    {
        mutex.lock();
        if (read(newsockfd, msgBuffer, 29) < 0) {
            error("Socket Read Error");
        }
        else {
            msgBuffer[29] = '\0';
            std::string msg(msgBuffer);
            if (msg == "99999999999999999999999999999") {
                queueBuildFlag = { true };
                return;
            }
            std::vector<cv::Vec3f> temp = extractData(msg);
            queue.push(temp);
            queueSize += 1;
        }
        if (write(newsockfd, "I got your message", 18) < 0) {
            error("Socket Write Error");
        }
        mutex.unlock();
    }
    void exposeQueue() { newData = { true }; }

    std::vector<cv::Vec3f> extractData(std::string localRadarData)
    {
        double lengthRdr, distanceRdr;
        try {
            lengthRdr = atof(localRadarData.substr(0, 14).c_str()) * 100.;
            distanceRdr = atof(localRadarData.substr(15, 29).c_str()) * 100.;
        }
        catch (const std::out_of_range& oor) {
            lengthRdr = 0.;
            distanceRdr = 0.;
        }
        return generatePointVec(static_cast<float>(lengthRdr),
            static_cast<float>(distanceRdr));
    }
    std::vector<cv::Vec3f> generatePointVec(float length, float distance)
    {
        float radarX, radarY, radarUpperZ, radarLowerZ;
        std::vector<cv::Vec3f> buffer;
        scaleZ = 1;//calculateScaleZ(distance);
        radarX = (distance + offsetX) * scaleX;
        radarUpperZ = ((length / 2.) + offsetZ) * scaleZ + lidarOffsetZ;
        radarLowerZ = ((-length / 2.) + offsetZ) * scaleZ + lidarOffsetZ;
        radarY = 0.0 + offsetY; // unknown for now
        if (length != 0.0 && distance != 0.0) {
            for (int i = ceil(radarLowerZ); i <= floor(radarUpperZ); i += 2) {
                buffer.push_back(cv::Vec3f(radarX, radarY, i));
                buffer.push_back(cv::Vec3f(radarX, radarY + 1, i));
                buffer.push_back(cv::Vec3f(radarX, radarY + 2, i));
            }
        }
        buffer.push_back(cv::Vec3f(std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN()));
        return buffer;
    }
    void generatePointVec(std::vector<cv::Vec3f>& radarBuffer)
    {
        float X, Y, upperZ, lowerZ;
        X = radarBuffer.front()[0];
        Y = radarBuffer.front()[1];
        lowerZ = radarBuffer.front()[2];
        upperZ = radarBuffer[radarBuffer.size() - 2][2];
        radarBuffer.clear();
        calculateScaleZ(X);
        X = X * scaleX;
        upperZ = upperZ * scaleZ + lidarOffsetZ;
        lowerZ = lowerZ * scaleZ + lidarOffsetZ;
        ;
        Y = 0.0 + offsetY; // unknown for now
        if (upperZ != 0.0 && X != 0.0) {
            for (int i = ceil(lowerZ); i <= floor(upperZ); i += 2) {
                radarBuffer.push_back(cv::Vec3f(X, Y, i));
                radarBuffer.push_back(cv::Vec3f(X, Y + 1, i));
                radarBuffer.push_back(cv::Vec3f(X, Y + 2, i));
            }
        }
        radarBuffer.push_back(cv::Vec3f(std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN()));
    }

    bool retrieve(std::vector<cv::Vec3f>& radarBuffer)
    {
        while (!mutex.try_lock()) {
            if (!queue.empty() && newData) {
                radarBuffer.clear();
                radarBuffer = std::move(queue.front());
                queue.pop();
                queueSize -= 1;
                newData = { false };
                mutex.unlock();
                return true;
            }
            if (!newData) {
                return false;
            }
        }
    };

    bool operator>>(std::vector<cv::Vec3f>& radarBuffer)
    {
        return retrieve(radarBuffer);
    };

    bool fitToLidar(std::vector<cv::Vec3f>& lidarPoints,
        std::vector<cv::Vec3f>& radarPoints, int lidarCycles)
    {
        float X, Y, upperZ, lowerZ, calcGroundZ, calcAvgCurbHeight, calcScaleZ,
            calcScaleX, calcLidarOffset;
        X = radarPoints.front()[0] * scaleX;
        Y = radarPoints.front()[1];
        lowerZ = radarPoints.front()[2] * scaleZ + lidarOffsetZ;
        upperZ = radarPoints[radarPoints.size() - 2][2] * scaleZ + lidarOffsetZ;
        if (isnanf(X)) {
            lidarCycles--;
            return 0;
        }
        if (lidarCycles > 179 && lidarCycles < 200) {
            findPointsInCurbRange(lidarPoints, 5.0, X);
        }
        else if (lidarCycles == 200) {
            // calcGroundZ = findGlobalMinAvgZ(lidarPoints, 50, findMinAvgZ(50));
            calcGroundZ = findMinAvgZ(50);
            calcAvgCurbHeight = findMaxAvgZ(200, calcGroundZ);
            float avgMinX = findMinAvgX(200);
            calcScaleZ = (calcAvgCurbHeight - calcGroundZ) / (upperZ - lowerZ);
            calcScaleX = X / avgMinX;
            calcLidarOffset = calcGroundZ - calcScaleZ * lowerZ;
            std::cout << "scaleZ: " << calcScaleZ << " scaleX: " << calcScaleX
                      << " avgMinX: " << avgMinX << std::endl;
            std::cout << "radarUpperZ: " << upperZ << " radarLowerZ: " << lowerZ
                      << " radarX: " << X << std::endl;
            std::cout << "groundZ: " << calcGroundZ
                      << " curbHeight: " << calcAvgCurbHeight
                      << " lidarOffsetZ: " << calcLidarOffset << std::endl;
            scaleZ = calcScaleZ;
            scaleX = calcScaleX;
            lidarOffsetZ = calcLidarOffset;
        }
        else if (lidarCycles > 200) {
            generatePointVec(radarPoints);
        }
        if (lidarCycles == 310) {
            std::cout << "scaleZ: " << scaleZ << " scaleX: " << scaleX << std::endl;
            std::cout << "radarUpperZ: " << upperZ << " radarLowerZ: " << lowerZ
                      << " radarX: " << X << std::endl;
        }
        return 1;
    }

    std::vector<cv::Vec3f> returnLidarPointsInRange()
    {
        return lidarPointsInRange;
    }
    float avgCurbX(std::vector<float> lidarXs)
    {
        return (quant(lidarXs, 0.75) + quant(lidarXs, 0.25)) / 2.;
    }
    void clearLidarPointsInRange() { lidarPointsInRange.clear(); }
    void findPointsInCurbRange(std::vector<cv::Vec3f>& lidarPoints,
        float percentTolerance, float distance)
    {
        std::vector<float> lidarXs;
        for (int i = 0; i < lidarPoints.size(); i++) {
            bool preFlag = isnanf(lidarPoints[i][0]) || isnanf(distance) || lidarPoints[i][0] < 0.0f;
            if (!preFlag) {
                if (isInPercentTolerance(distance, lidarPoints[i][0],
                        percentTolerance)) {
                    lidarXs.push_back(lidarPoints[i][0]);
                }
            }
        }
        float curbX = avgCurbX(lidarXs);
        for (int i = 0; i < lidarPoints.size(); i++) {
            bool preFlag = isnanf(lidarPoints[i][0]) || isnanf(distance) || lidarPoints[i][0] < 0.0f;
            if (!preFlag) {
                if (isInPercentTolerance(curbX, lidarPoints[i][0], 1.0)) {
                    lidarPointsInRange.push_back(lidarPoints[i]);
                }
            }
        }
    }

    float findGlobalMinAvgZ(std::vector<cv::Vec3f>& lidarPoints, int numsamples,
        float localGroundZ)
    {
        int size = 0;
        float globalMin = 0;
        for (int i = 0; i < lidarPoints.size(); i++) {
            bool preFlag = isnanf(lidarPoints[i][0]) || lidarPoints[i][0] < 0.0f;
            if (!preFlag && (lidarPoints[i][2] <= localGroundZ)) {
                globalMin += lidarPoints[i][2];
                size++;
            }
        }
        return globalMin / static_cast<float>(size);
    }
    float findMinAvgZ(int numSamples)
    {
        std::vector<float> minZVec;
        int count = 0;
        for (int i = 0; i < lidarPointsInRange.size(); i++) {
            minZVec.push_back(lidarPointsInRange[i][2]);
            if (lidarPointsInRange[i][2] < -50.) {
                firstObject.push_back(lidarPointsInRange[i]);
            }
        }
        float minZ = 0.0f;
        int minZSize = minZVec.size();
        int size = std::min(numSamples, minZSize);
        for (int i = 0; i < size; i++) {
            int minIndex = std::distance(
                minZVec.begin(), std::min_element(minZVec.begin(), minZVec.end()));
            std::cout << "minZ: " << minZVec[minIndex] << std::endl;
            minZ += minZVec[minIndex];
            minZVec.erase(minZVec.begin() + minIndex);
        }
        return (minZ / static_cast<float>(size));
    }
    float findMaxAvgZ(int numSamples, float ground)
    {
        std::vector<float> maxZVec;
        int count = 0;
        for (int i = 0; i < lidarPointsInRange.size(); i++) {
            if ((lidarPointsInRange[i][2] <= (ground + 40)) && (lidarPointsInRange[i][2] >= ground)) {
                maxZVec.push_back(lidarPointsInRange[i][2]);
                secondObject.push_back(lidarPointsInRange[i]);
            }
        }
        float maxZ = 0.0f;
        int maxZVecSize = maxZVec.size();
        int size = std::min(numSamples, maxZVecSize);
        for (int i = 0; i < size; i++) {
            int maxIndex = std::distance(
                maxZVec.begin(), std::max_element(maxZVec.begin(), maxZVec.end()));
            maxZ += maxZVec[maxIndex];
            maxZVec.erase(maxZVec.begin() + maxIndex);
        }
        return (maxZ / static_cast<float>(size));
    }
    float findMinAvgX(int numSamples)
    {
        std::vector<float> minXVec;
        int count = 0;
        for (int i = 0; i < lidarPointsInRange.size(); i++) {
            minXVec.push_back(lidarPointsInRange[i][0]);
        }
        float minX = 0.0f;
        int minXSize = minXVec.size();
        int size = std::min(numSamples, minXSize);
        for (int i = 0; i < size; i++) {
            int minIndex = std::distance(
                minXVec.begin(), std::min_element(minXVec.begin(), minXVec.end()));
            minX += minXVec[minIndex];
            minXVec.erase(minXVec.begin() + minIndex);
        }
        return (minX / static_cast<float>(size));
    }
    float calculateScaleZ(float rawRadarX)
    {
        if (!isnanf(rawRadarX) || rawRadarX) {
            return (74.5 * powf(rawRadarX, -1.12));
        }
        else {
            return 0;
        }
    }
    bool cycle210 = true;
    bool cycle430 = true;
    float firstObjectInitialY, secondObjectInitialY, secondObjectFinalY,
        distanceTraveled;
    bool fitToLidarY(std::vector<cv::Vec3f>& lidarPoints,
        std::vector<cv::Vec3f>& radarPoints, int cycles)
    {
        if (isnanf(radarPoints.front()[0])) {
            cycles--;
            return 0;
        }
        if ((cycles > 189 && cycles < 210) || (cycles > 389 && cycles < 410)) {
            findPointsInRange(lidarPoints, 40, radarPoints.front()[0]);
        }
        else if (cycles == 210) {
            if (cycle210) {
                firstObjectInitialY = findObjectYByHeight(lidarPointsInRange, 15, -35, firstObject, 0, 170, true);
                secondObjectInitialY = findObjectYByHeight(lidarPointsInRange, 15, -20, secondObject, 1, 270, false);
                lidarPointsInRange.clear();
                cycle210 = false;
            }
        }
        else if (cycles == 410) {
            firstObject.clear();
            secondObject.clear();
            if (cycle430) {
                secondObjectFinalY = findObjectYByHeight(lidarPointsInRange, 15, -20, secondObject, 1, 220, false);
                offsetY = (secondObjectInitialY - secondObjectFinalY) - firstObjectInitialY;
                std::cout << "offsetY: " << offsetY << " firstObjectInitialY: " << firstObjectInitialY << " secondObjectInitialY: " << secondObjectInitialY << " secondObjectFinalY: " << secondObjectFinalY << std::endl;
                cycle430 = false;
            }
        }
        return 1;
    }
    void findPointsInRange(std::vector<cv::Vec3f>& lidarPoints,
        float percentTolerance, float distance)
    {
        for (int i = 0; i < lidarPoints.size(); i++) {
            bool preFlag = isnanf(lidarPoints[i][0]) || isnanf(distance) || lidarPoints[i][0] < 0.0f;
            if (!preFlag) {
                if (isInPercentTolerance(distance, lidarPoints[i][0], percentTolerance)) {
                    lidarPointsInRange.push_back(lidarPoints[i]);
                }
            }
        }
    }
    std::vector<cv::Vec3f> returnFirstObject() { return firstObject; }
    std::vector<cv::Vec3f> returnSecondObject() { return secondObject; }
    float findObjectYByHeight(std::vector<cv::Vec3f>& lidarPoints, float percentTolerance, float height, std::vector<cv::Vec3f>& object, int limIndex, float lim, bool toggle)
    {
        float y = 0;
        int cycles = 0;
        for (int i = 0; i < lidarPoints.size(); i++) {
            bool preFlag = isnanf(lidarPoints[i][1]);
            if (!preFlag) {
                if (isInPercentTolerance(height, lidarPoints[i][2],
                        percentTolerance)
                    && lidarPoints[i][limIndex] < lim) {
                    if (!toggle) {
                        object.push_back(lidarPoints[i]);
                        y += lidarPoints[i][1];
                        cycles++;
                    }
                    else {
                        if (lidarPoints[i][1] < 100) {
                            object.push_back(lidarPoints[i]);
                            y += lidarPoints[i][1];
                            cycles++;
                        }
                    }
                }
            }
        }
        std::cout << "y: " << y << " cycles: " << cycles << std::endl;
        return y / static_cast<float>(cycles);
    }

    void closeThread(std::thread* thread)
    {
        if (thread && thread->joinable()) {
            thread->join();
            thread->~thread();
            delete thread;
            thread = nullptr;
        }
    }
    void close()
    {
        run = false;
        closeThread(threadQueueRead);
        closeThread(threadQueueWrite);
    }

    void error(const char* msg)
    {
        perror(msg);
        exit(1);
    }
};
}