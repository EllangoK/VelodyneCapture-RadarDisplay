#include <vector>
#include <string>
#include <math.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <opencv2/opencv.hpp>

namespace radar {
struct Boundary {
    float length, distance;
};
struct DistanceDensity {
    double distance;
    double density;

    bool operator<(const DistanceDensity& a) const
    {
        return distance < a.distance;
    }
};
class RadarPacket {
private:
    long long timeStartSend, timeReceived, timeProcessed; // microseconds
    std::vector<float> lengthVec;
    std::vector<float> distanceVec;
    int radarBoundaryIndex;
    Boundary boundary;
    float scaleX;
    float offsetX, offsetY, offsetZ, lidarOffsetZ;
public:
    RadarPacket(long long timeStartSend, long long timeReceived,
        std::vector<std::string>& socketData)
    {
        this->timeStartSend = timeStartSend;
        this->timeReceived = timeReceived;
        extractData(socketData);
        radarBoundaryIndex = distanceVec.size() - 1;
    }
    RadarPacket()
    {
        lengthVec.push_back(std::numeric_limits<float>::quiet_NaN());
        distanceVec.push_back(std::numeric_limits<float>::quiet_NaN());
        radarBoundaryIndex = 0;
    }
    void setCalibrationParams(std::vector<float>& params)
    {
        scaleX = params[0];
        lidarOffsetZ = params[1];
        offsetX = params[2];
        offsetY = params[3];
        offsetZ = params[4];
    }
    std::vector<float> getCalibrationParams()
    {
        std::vector<float> params = { scaleX, lidarOffsetZ, offsetX, offsetY,
            offsetZ };
        return params;
    }

    std::vector<float> extractData(std::vector<std::string> socketData)
    {
        for (std::string data : socketData) {
            lengthVec.push_back((float)atof(data.substr(0, 14).c_str()) * 100.);
            distanceVec.push_back((float)atof(data.substr(15, 29).c_str()) * 100.);
        }
    }

    std::vector<float> getDistanceVec() { return distanceVec; }
    std::vector<float> getLengthVec() { return lengthVec; }
    void setBoundary(float length, float distance) { boundary = {length, distance}; }
    Boundary getBoundary() { return boundary; }
    int getBoundaryIndex() { return radarBoundaryIndex; }
    int size() { return distanceVec.size(); }

    std::vector<cv::Vec3f> generatePointVec(int boundaryIndex,
        std::vector<cv::Vec3f>& bound)
    {
        float x, y, zLower, zUpper;
        float length = lengthVec[boundaryIndex];
        float distance = distanceVec[boundaryIndex];
        float scaleZ = calculateScaleZ(distance, false);
        x = (distance + offsetX) * scaleX;
        y = 0.0 + offsetY;
        zLower = (-length / 2. + offsetZ) * scaleZ + lidarOffsetZ;
        zUpper = (length / 2. + offsetZ) * scaleZ + lidarOffsetZ;
        if (std::isinf(zLower) || isnanf(zLower) || isnanf(x) || x > 2000) {
            x = std::numeric_limits<float>::quiet_NaN();
            bound.push_back(cv::Vec3f(x, x, x));
            return bound;
        }
        for (int i = ceil(zLower); i <= floor(zUpper); i += 3) {
            bound.push_back(cv::Vec3f(x, y, i));
        }
        return bound;
    }
    std::vector<cv::Vec3f> generatePointVec(int boundaryIndex)
    {
        std::vector<cv::Vec3f> bound;
        float x, y, zLower, zUpper;
        float length = lengthVec[boundaryIndex];
        float distance = distanceVec[boundaryIndex];
        float scaleZ = calculateScaleZ(distance, false);
        x = (distance + offsetX) * scaleX;
        y = 0.0 + offsetY;
        zLower = (-length / 2. + offsetZ) * scaleZ + lidarOffsetZ;
        zUpper = (length / 2. + offsetZ) * scaleZ + lidarOffsetZ;
        if (std::isinf(zLower) || isnanf(zLower) || isnanf(x) || x > 2000) {
            x = std::numeric_limits<float>::quiet_NaN();
            bound.push_back(cv::Vec3f(x, x, x));
            return bound;
        }
        //std::cout << "From Original x: " << x << " y: " << y << " zLower: " << zLower << " zUpper: " << zUpper << std::endl;
        for (int i = ceil(zLower); i <= floor(zUpper); i += 5) {
            bound.push_back(cv::Vec3f(x, y, i));
        }
        return bound;
    }
    std::vector<cv::Vec3f> generatePointVec(float length, float distance)
    {
        std::vector<cv::Vec3f> bound;
        float x, y, zLower, zUpper;
        float scaleZ = calculateScaleZ(distance, true);
        x = (distance + offsetX) * scaleX;
        y = 0.0 + offsetY;
        zLower = (-length / 2. + offsetZ) * scaleZ + lidarOffsetZ;
        zUpper = (length / 2. + offsetZ) * scaleZ + lidarOffsetZ;
        if (std::isinf(zLower) || isnanf(zLower) || isnanf(x) || x > 2000) {
            x = std::numeric_limits<float>::quiet_NaN();
            bound.push_back(cv::Vec3f(x, x, x));
            return bound;
        }
        //std::cout << "From Kernel x: " << x << " y: " << y << " zLower: " << zLower << " zUpper: " << zUpper << std::endl;
        for (int i = ceil(zLower); i <= floor(zUpper); i += 5) {
            bound.push_back(cv::Vec3f(x, y, i));
        }
        return bound;
    }
    std::vector<cv::Vec3f> rescalePointVec(
        std::vector<cv::Vec3f>& radarPointCloud, std::vector<float>& params,
        float scaleZ)
    {
        float prevScaleX, prevScaleZ;
        float prevOffsetX, prevOffsetY, prevOffsetZ, prevLidarOffsetZ;
        std::vector<float> prevParams = getCalibrationParams();
        prevScaleX = prevParams[0];
        prevLidarOffsetZ = prevParams[1];
        prevOffsetX = prevParams[2];
        prevOffsetY = prevParams[3];
        prevOffsetZ = prevParams[4];
        prevScaleZ = calculateScaleZ(
            (radarPointCloud.front()[0] / prevScaleX - prevOffsetX));
        setCalibrationParams(params);
        float x, y, zLower, zUpper;
        x = (radarPointCloud.front()[0] / prevScaleX - prevOffsetX) + offsetX * scaleX;
        y = radarPointCloud.front()[1] - prevOffsetY + offsetY;
        zLower = (radarPointCloud.back()[2] - prevLidarOffsetZ / prevScaleZ - prevOffsetZ) + offsetZ * scaleZ + lidarOffsetZ;
        zUpper = (radarPointCloud.front()[2] - prevLidarOffsetZ / prevScaleZ - prevOffsetZ) + offsetZ * scaleZ + lidarOffsetZ;
        for (int i = ceil(zLower); i <= floor(zUpper); i += 2) {
            radarPointCloud.push_back(cv::Vec3f(x, y, i));
        }
        return radarPointCloud;
    }
    std::vector<cv::Vec3f> generateAllPointVec()
    {
        std::vector<cv::Vec3f> allBoundaries;
        if (size() < 2) {
            allBoundaries.push_back(cv::Vec3f(std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()));
            return allBoundaries;
        }
        for (int i = 0; i < size() - 1; i++) {
            generatePointVec(i, allBoundaries);
        }
        return allBoundaries;
    }

    float calculateScaleZ(float distance, bool enabled = true)
    {
        return enabled ? (74.5 * powf(distance, -1.12)) : 1;
    }

    bool isInPercentTolerance(float measured, float actual,
        float percentTolerance)
    {
        return (fabs((measured - actual) / actual) * 100. <= percentTolerance);
    }

    std::vector<cv::Vec3f> findBoundary(std::deque<radar::RadarPacket>& packets, double percentTolerance)
    {
        if (size() < 2) {
            std::vector<cv::Vec3f> NaN;
            NaN.push_back(cv::Vec3f(std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()));
            return NaN;
        }
        int indices[size() + 1] = {};
        for (int i = 0; i < packets.size(); i++) {
            indices[findBoundaryIndex(packets[i], percentTolerance)] += 1;
        }
        const int N = sizeof(indices) / sizeof(int);
        std::vector<float> temp;
        if (*std::max_element(indices, indices + N) == indices[size()]) {
            indices[size()] = 0;
        }
        return generatePointVec(std::distance(indices, std::max_element(indices, indices + N)));
    }

    int findBoundaryIndex(radar::RadarPacket& prev, double percentTolerance)
    {
        if (size() < 2) {
            return 0;
        }
        std::vector<float> prevDistanceVec = prev.getDistanceVec();
        int prevBoundaryIndex = prev.getBoundaryIndex();
        if (matchHighOnOldRadarBoundary(prev, percentTolerance)) {
            return radarBoundaryIndex;
        }
        if (matchAnyOnOldRadarBoundary(prev, percentTolerance)) {
            return radarBoundaryIndex;
        }
        for (int i = 0; i < prevDistanceVec.size() - 1; i++) {
            if (isInPercentTolerance(distanceVec.back(), prevDistanceVec[i], percentTolerance)) {
                //std::cout << "Scenario Two: " << distanceVec[i] << " " << prevDistanceVec[i] << std::endl;
                radarBoundaryIndex = i;
                return radarBoundaryIndex;
            }
        }
        //std::cout << "No Matches" << std::endl;
        return int(lengthVec.size());
    }
    bool matchHighOnOldRadarBoundary(radar::RadarPacket& prev, double percentTolerance)
    {
        if (isInPercentTolerance(distanceVec.back(),
                prev.getDistanceVec()[prev.getBoundaryIndex()], percentTolerance)) {
            //std::cout << "Matched Old: " << distanceVec.back() << " " << prev.getDistanceVec()[prev.getBoundaryIndex()] << std::endl;
            radarBoundaryIndex = distanceVec.size() - 1;
            return true;
        }
        return false;
    }
    bool matchAnyOnOldRadarBoundary(radar::RadarPacket& prev, double percentTolerance)
    {
        for (int i = 0; i < distanceVec.size() - 1; i++) {
            if (isInPercentTolerance(distanceVec[i],
                    prev.getDistanceVec()[prev.getBoundaryIndex()], percentTolerance)) {
                if (distanceVec.back() <= distanceVec[i]) {
                    //std::cout << "Scenario One: ";
                    radarBoundaryIndex = distanceVec.size() - 1;
                    //std::cout << "MinHC: " << distanceVec[radarBoundaryIndex] << std::endl;
                    return true;
                }
                else {
                    radarBoundaryIndex = i;
                    //std::cout << distanceVec[radarBoundaryIndex] << " " << prev.getDistanceVec()[prev.getBoundaryIndex()] << std::endl;
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<cv::Vec3f> generateBoundaryFromKernel(std::deque<radar::RadarPacket>& packets, float bandwith)
    {
        std::vector<DistanceDensity> ddX;
        std::vector<float> xVec, peaks;
        xVec = concatenatePrevBoundaries(packets);
        ddX = generateDensityVector(packets, xVec, bandwith);
        if (ddX.size()) {
            findPeaks(ddX, peaks);
            std::cout << "peaks: ";
            for (int i = 0; i < peaks.size(); i++) {
        		std::cout << peaks.at(i) << ' ';
        	}
            std::cout << std::endl;
            if (peaks.size()) {
                float approxLength = 0;
                for (int i = packets.size()-2; i >= 0 ; i--) {
                    if (isInPercentTolerance(peaks[0], packets[i].getBoundary().distance, 10)) {
                        approxLength = packets[i].getBoundary().length;
                        break;
                    }
                }
                if (approxLength == 0) {
                    int index = (int)(std::min_element(distanceVec.begin(), distanceVec.end()) - distanceVec.begin());
                    boundary = { lengthVec[index], distanceVec[index] };
                    std::cout << "no length : " << lengthVec[index] << " " << distanceVec[index] << std::endl;
                    return generatePointVec(lengthVec[index], distanceVec[index]);
                }
                boundary = { approxLength, peaks[0] };
                return generatePointVec(approxLength, peaks[0]);
            }
            else {
                std::cout << "no peaks" << std::endl;
            }
        }
        else {
            std::cout << "cheated: " << std::endl;
            int index = (int)(std::min_element(distanceVec.begin(), distanceVec.end()) - distanceVec.begin());
            boundary = { lengthVec[index], distanceVec[index] };
            return generatePointVec(lengthVec[index], distanceVec[index]);
        }
    }
    std::vector<float> concatenatePrevBoundaries(std::deque<radar::RadarPacket>& packets)
    {
        std::vector<float> xVec;
        for (int i = 0; i < packets.size(); i++) {
            std::vector<float> temp = packets[i].getDistanceVec();
            if (isInPercentTolerance(temp.back(), packets.back().getBoundary().distance, 10)) {
                xVec.push_back(temp.back());
            }
            for (int j = 0; j < i / 2.; j++) {
                xVec.insert(xVec.end(), temp.begin(), temp.end());
            }
            std::move(temp.begin(), temp.end(), std::back_inserter(xVec));
        }
        xVec.erase(std::remove_if(xVec.begin(), xVec.end(), isnanf), xVec.end());
        return xVec;
    }
    std::vector<DistanceDensity> generateDensityVector(std::deque<radar::RadarPacket>& packets, std::vector<float>& xVec, float bandwith)
    {
        std::vector<DistanceDensity> densityVec;
        for (int i = 0; i < (int)(*std::max_element(xVec.begin(), xVec.end())) * 2; i += 1) {
            float density = probabilityDensityFunction(i / 2., xVec, bandwith);
            if (int(density) != 0) {
                densityVec.push_back({ i / 2., density });
            }
        }
        return densityVec;
    }
    void findPeaks(std::vector<DistanceDensity>& ddVec, std::vector<float>& peaks)
    {
        std::sort(ddVec.begin(), ddVec.end());
        int numPeaks = std::min(size() + 1, (int)ddVec.size());
        for (int i = ddVec.size() - numPeaks; i < ddVec.size(); i++) {
            peaks.push_back(ddVec[i].distance);
        }
    }
    double probabilityDensityFunction(double x, std::vector<float>& xVec, double bandwith)
    {
        double sum = 0;
        for (int i = 0; i < xVec.size(); i++) {
            sum += standardNormalKernel((x - xVec[i]) / bandwith);
        }
        sum /= xVec.size() * bandwith;
        if (isnanf(sum)) {
            return 0;
        }
        return sum;
    }
    double standardNormalKernel(double x)
    {
        return exp(-0.5 * pow(x, 2)) / sqrt(2 * CV_PI);
    }
};

class RadarServer {
private:
    int sockfd, newsockfd, servlen, n;
    socklen_t clilen;
    struct sockaddr_un cli_addr, serv_addr;
    char buf[29];
    std::string socketPath;

    std::mutex mutex;
    std::thread* socketThread = nullptr;
    std::atomic_bool socketRun = { true };
    std::queue<radar::RadarPacket> queue;
    std::atomic_int queueSize = { 0 };

    std::vector<float> params;

public:
    RadarServer(std::string socketPath, std::vector<float> params)
    {
        this->socketPath = socketPath;
        this->params = params;
        createServer();
        threadSocket();
    }
    ~RadarServer()
    {
        closeSocket();
    }

    void error(const char* msg)
    {
        perror(msg);
        exit(0);
    }
    bool isEmpty() { return queue.empty(); }
    bool isRun() { return socketRun; }
    int getQueueSize() { return queueSize; }

    void createServer()
    {
        sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
        if (sockfd < 0) {
            error("Opening Socket Error");
        }
        bzero((char*)&serv_addr, sizeof(serv_addr));
        serv_addr.sun_family = AF_UNIX;
        strcpy(serv_addr.sun_path, socketPath.c_str());
        if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            error("Binding Error");
        }
        listen(sockfd, 4);
        clilen = sizeof(cli_addr);
        newsockfd = accept(sockfd, (struct sockaddr*)&cli_addr, &clilen);
        if (newsockfd < 0) {
            error("Connection Accept Error");
        }
        bzero(buf, 29);
    }
    void closeSocket()
    {
        if (!socketRun) {
            closeThread(socketThread);
            close(newsockfd);
            close(sockfd);
        }
    }
    void threadSocket()
    {
        auto funcReadSocket = std::bind(&RadarServer::readSocket, this);
        unsigned int updateTime = 1;
        socketThread = new std::thread([funcReadSocket, updateTime]() {
            while (true) {
                auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(updateTime);
                funcReadSocket();
                std::this_thread::sleep_until(x);
            }
        });
    }
    void readSocket()
    {
        if (!socketRun) {
            return;
        }
        switch (read(newsockfd, buf, 29)) {
        case -1:
            error("Socket Read Error");
        case 0:
            return;
        case 29:
            buf[29] = '\0';
            std::string msg(buf);
            if (msg.substr(0, 13) == "ClientClosed:") {
                socketRun = { false };
                return;
            }
            if (msg.substr(0, 5) == "Start") {
                std::vector<std::string> packetPayload;
                long long packetStart, packetStop;
                int packetLen = std::stoi(msg.substr(26, 3));
                RadarPacket data;
                if (packetLen == 0) {
                    data = RadarPacket();
                    read(newsockfd, buf, 29);
                }
                else {
                    packetStart = std::stoll(msg.substr(6, 16));
                    for (int i = 0; i < packetLen; i++) {
                        read(newsockfd, buf, 29);
                        buf[29] = '\0';
                        packetPayload.push_back(std::string(buf));
                    }
                    read(newsockfd, buf, 29);
                    packetStop = std::stoll(msg.substr(12, 16));
                    data = RadarPacket(packetStart, packetStop, packetPayload);
                }
                data.setCalibrationParams(params);
                while (true) {
                    if (mutex.try_lock()) {
                        queue.push(data);
                        queueSize += 1;
                        mutex.unlock();
                        break;
                    }
                }
            }
        }
    }

    void retrieve(radar::RadarPacket& packet)
    {
        while (true) {
            if (mutex.try_lock() && !queue.empty()) {
                packet = queue.front();
                queue.pop();
                queueSize -= 1;
                mutex.unlock();
                break;
            }
        }
    };
    void operator>>(radar::RadarPacket& packet) { return retrieve(packet); };

    void closeThread(std::thread* thread)
    {
        if (thread && thread->joinable()) {
            thread->join();
            thread->~thread();
            delete thread;
            thread = nullptr;
        }
    }
};

class RadarParamGen {
private:
    float offsetX, offsetY, offsetZ, lidarOffsetZ = 0;
    float avgCurbHeight = 0, groundZ = 0, scaleZ = 1, scaleX = 1;
    float firstObjectInitialY, secondObjectInitialY, secondObjectFinalY,
        distanceTraveled;
    std::vector<cv::Vec3f> lidarPointsInRange, firstObject, secondObject;

public:
    RadarParamGen() {}

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
    bool isInPercentTolerance(float measured, float actual,
        float percentTolerance)
    {
        return (fabs((measured - actual) / actual) * 100. <= percentTolerance);
    }

    void fitToLidar(std::vector<cv::Vec3f>& lidarPoints,
        std::vector<cv::Vec3f>& radarPoints, int lidarCycles)
    {
        if (isnanf(radarPoints.front()[0])) {
            return;
        }
        float X, Y, upperZ, lowerZ, calcGroundZ, calcAvgCurbHeight, calcScaleZ,
            calcScaleX, calcLidarOffset;
        X = radarPoints.front()[0] * scaleX;
        Y = radarPoints.front()[1];
        lowerZ = radarPoints.front()[2] * scaleZ + lidarOffsetZ;
        upperZ = radarPoints[radarPoints.size() - 2][2] * scaleZ + lidarOffsetZ;
        if (lidarCycles > 179 && lidarCycles < 200) {
            findPointsInCurbRange(lidarPoints, 5.0, X);
        }
        else if (lidarCycles == 200) {
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
        if (lidarCycles == 310) {
            std::cout << "scaleZ: " << scaleZ << " scaleX: " << scaleX << std::endl;
            std::cout << "radarUpperZ: " << upperZ << " radarLowerZ: " << lowerZ
                      << " radarX: " << X << std::endl;
        }
    }

    std::vector<cv::Vec3f> returnLidarPointsInRange()
    {
        return lidarPointsInRange;
    }
    void clearLidarPointsInRange() { lidarPointsInRange.clear(); }

    void findPointsInRange(std::vector<cv::Vec3f>& lidarPoints,
        float percentTolerance, float distance)
    {
        for (int i = 0; i < lidarPoints.size(); i++) {
            if (!isnanf(lidarPoints[i][0]) && isInPercentTolerance(distance, lidarPoints[i][0], percentTolerance)) {
                lidarPointsInRange.push_back(lidarPoints[i]);
            }
        }
    }
    float avgCurbX(std::vector<float> lidarXs)
    {
        return (quant(lidarXs, 0.75) + quant(lidarXs, 0.25)) / 2.;
    }
    void findPointsInCurbRange(std::vector<cv::Vec3f>& lidarPoints,
        float percentTolerance, float distance)
    {
        std::vector<float> lidarXs;
        for (int i = 0; i < lidarPoints.size(); i++) {
            if (!isnanf(lidarPoints[i][0]) && isInPercentTolerance(distance, lidarPoints[i][0], percentTolerance)) {
                lidarXs.push_back(lidarPoints[i][0]);
            }
        }
        float curbX = avgCurbX(lidarXs);
        for (int i = 0; i < lidarPoints.size(); i++) {
            if (!isnanf(lidarPoints[i][0]) && isInPercentTolerance(curbX, lidarPoints[i][0], 1.0)) {
                lidarPointsInRange.push_back(lidarPoints[i]);
            }
        }
    }

    float findGlobalMinAvgZ(std::vector<cv::Vec3f>& lidarPoints, int numsamples,
        float localGroundZ)
    {
        int size = 0;
        float globalMin = 0;
        for (int i = 0; i < lidarPoints.size(); i++) {
            if (!!isnanf(lidarPoints[i][0]) && (lidarPoints[i][2] <= localGroundZ)) {
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

    void fitToLidarY(std::vector<cv::Vec3f>& lidarPoints,
        std::vector<cv::Vec3f>& radarPoints, int cycles)
    {
        if (isnanf(radarPoints.front()[0])) {
            return;
        }
        if ((cycles > 189 && cycles < 210) || (cycles > 389 && cycles < 410)) {
            findPointsInRange(lidarPoints, 40, radarPoints.front()[0]);
        }
        else if (cycles == 210) {
            firstObjectInitialY = findObjectYByHeight(lidarPointsInRange, 15, -35,
                firstObject, 0, 170, true);
            secondObjectInitialY = findObjectYByHeight(lidarPointsInRange, 15, -20,
                secondObject, 1, 270, false);
            lidarPointsInRange.clear();
        }
        else if (cycles == 410) {
            firstObject.clear();
            secondObject.clear();
            secondObjectFinalY = findObjectYByHeight(lidarPointsInRange, 15, -20,
                secondObject, 1, 220, false);
            offsetY = (secondObjectInitialY - secondObjectFinalY) - firstObjectInitialY;
            std::cout << "offsetY: " << offsetY
                      << " firstObjectInitialY: " << firstObjectInitialY
                      << " secondObjectInitialY: " << secondObjectInitialY
                      << " secondObjectFinalY: " << secondObjectFinalY << std::endl;
        }
    }

    std::vector<cv::Vec3f> returnFirstObject() { return firstObject; }
    std::vector<cv::Vec3f> returnSecondObject() { return secondObject; }
    float findObjectYByHeight(std::vector<cv::Vec3f>& lidarPoints,
        float percentTolerance, float height,
        std::vector<cv::Vec3f>& object, int limIndex,
        float lim, bool toggle)
    {
        float y = 0;
        int cycles = 0;
        for (int i = 0; i < lidarPoints.size(); i++) {
            if (!isnanf(lidarPoints[i][1])) {
                if (isInPercentTolerance(height, lidarPoints[i][2], percentTolerance) && lidarPoints[i][limIndex] < lim) {
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
};
}