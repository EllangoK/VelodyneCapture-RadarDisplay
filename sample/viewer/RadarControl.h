#include <vector>
#include <string>

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
class RadarPacket {
 private:
  long long timeStartSend, timeReceived, timeProcessed;  // microseconds
  std::vector<float> lengthVec;
  std::vector<float> distanceVec;
  std::vector<cv::Vec3f> boundary;
  float scaleX;
  float offsetX, offsetY, offsetZ, lidarOffsetZ;

 public:
  RadarPacket(long long timeStartSend, long long timeReceived,
              std::vector<std::string>& socketData) {
    this->timeStartSend = timeStartSend;
    this->timeReceived = timeReceived;
  }
  RadarPacket() {
    boundary.push_back(cv::Vec3f(std::numeric_limits<float>::quiet_NaN(),
                                 std::numeric_limits<float>::quiet_NaN(),
                                 std::numeric_limits<float>::quiet_NaN()));
    lengthVec.push_back(std::numeric_limits<float>::quiet_NaN());
    distanceVec.push_back(std::numeric_limits<float>::quiet_NaN());
  }
  void setCalibrationParams(std::vector<float>& params) {
    scaleX = params[0];
    lidarOffsetZ = params[1];
    offsetX = params[2];
    offsetY = params[3];
    offsetZ = params[4];
  }
  std::vector<float> getCalibrationParams() {
    std::vector<float> params = {scaleX, lidarOffsetZ, offsetX, offsetY,
                                 offsetZ};
    return params;
  }

  std::vector<float> extractData(std::vector<std::string>& socketData) {
    for (std::string data : socketData) {
      lengthVec.push_back((float)atof(data.substr(0, 14).c_str()) * 100.);
      distanceVec.push_back((float)atof(data.substr(15, 29).c_str()) * 100.);
    }
  }

  std::vector<float> getDistanceVec() { return distanceVec; }
  std::vector<float> getLengthVec() { return lengthVec; }
  std::vector<cv::Vec3f> getBoundary() { return boundary; }

  std::vector<cv::Vec3f> generatePointVec(int boundaryIndex,
                                          std::vector<cv::Vec3f>& boundary) {
    float x, y, zLower, zUpper;
    float length = lengthVec[boundaryIndex];
    float distance = distanceVec[boundaryIndex];
    float scaleZ = calculateScaleZ(distance);
    x = (distance + offsetX) * scaleX;
    y = 0.0 + offsetY;
    zLower = (length / 2. + offsetZ) * scaleZ + lidarOffsetZ;
    zUpper = (-length / 2. + offsetZ) * scaleZ + lidarOffsetZ;
    for (int i = ceil(zLower); i <= floor(zUpper); i += 2) {
      boundary.push_back(cv::Vec3f(x, y, i));
    }
    return boundary;
  }
  std::vector<cv::Vec3f> rescalePointVec(
      std::vector<cv::Vec3f>& radarPointCloud, std::vector<float>& params,
      float scaleZ) {
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
    x = (radarPointCloud.front()[0] / prevScaleX - prevOffsetX) +
        offsetX * scaleX;
    y = radarPointCloud.front()[1] - prevOffsetY + offsetY;
    zLower = (radarPointCloud.back()[2] - prevLidarOffsetZ / prevScaleZ -
              prevOffsetZ) +
             offsetZ * scaleZ + lidarOffsetZ;
    zUpper = (radarPointCloud.front()[2] - prevLidarOffsetZ / prevScaleZ -
              prevOffsetZ) +
             offsetZ * scaleZ + lidarOffsetZ;
    for (int i = ceil(zLower); i <= floor(zUpper); i += 2) {
      radarPointCloud.push_back(cv::Vec3f(x, y, i));
    }
    return radarPointCloud;
  }
  std::vector<cv::Vec3f> generateAllPointVec() {
    std::vector<cv::Vec3f> allBoundaries;
    if (lengthVec.size() == 1) {
      allBoundaries.push_back(
          cv::Vec3f(std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN()));
    }
    for (int i = 0; i < lengthVec.size() - 1; i++) {
      generatePointVec(i, allBoundaries);
    }
    return allBoundaries;
  }

  float calculateScaleZ(float distance, bool enabled = true) {
    return enabled ? (74.5 * powf(distance, -1.12)) : 1;
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
  std::thread* closeSocketThread = nullptr;
  std::atomic_bool socketRun = {true};
  std::queue<radar::RadarPacket> queue;
  std::atomic_int queueSize = {0};

  std::vector<float> params;

 public:
  RadarServer(std::string socketPath, std::vector<float> params) {
    this->socketPath = socketPath;
    this->params = params;
    createServer();
  }
  ~RadarServer() {
    closeSocket();
    closeThread(closeSocketThread);
  }
  void error(const char*);

  void createServer() {
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
  void closeSocket() {
    if (!socketRun) {
      closeThread(socketThread);
      close(newsockfd);
      close(sockfd);
    }
  }
  void threadSocket() {
    auto funcReadSocket = std::bind(&RadarServer::readSocket, this);
    unsigned int updateTime = 20;
    socketThread = new std::thread([funcReadSocket, updateTime]() {
      while (true) {
        auto x = std::chrono::steady_clock::now() +
                 std::chrono::milliseconds(updateTime);
        funcReadSocket();
        std::this_thread::sleep_until(x);
      }
    });
    auto funcCloseSocket = std::bind(&RadarServer::closeSocket, this);
    socketThread = new std::thread([funcReadSocket, updateTime]() {
      while (true) {
        auto x = std::chrono::steady_clock::now() +
                 std::chrono::milliseconds(updateTime);
        funcReadSocket();
        std::this_thread::sleep_until(x);
      }
    });
  }
  void readSocket() {
    switch (read(newsockfd, buf, 29)) {
      case -1:
        error("Socket Read Error");
      case 0:
        socketRun = {false};
      case 29:
        buf[29] = '\0';
        std::string msg(buf);
        if (msg == "99999999999999999999999999999") {
          socketRun = {false};
          return;
        }
        if (msg.substr(0, 5) == "Start") {
          std::vector<std::string> packetPayload;
          long long packetStart, packetStop;
          packetStart = std::stoll(msg.substr(6, 16));
          int packetLen = std::stoi(msg.substr(26, 3));
          for (int i = 0; i < packetLen; i++) {
            read(newsockfd, buf, 29);
            buf[29] = '\0';
            packetPayload.push_back(std::string(buf));
          }
          read(newsockfd, buf, 29);
          packetStop = std::stoll(msg.substr(12, 16));
          RadarPacket data =
              RadarPacket(packetStart, packetStop, packetPayload);
          data.setCalibrationParams(params);
          while (true) {
            if (mutex.try_lock()) {
              queue.push(data);
              queueSize += 1;
              mutex.unlock();
            }
          }
        }
    }
  }

  void retrieve(radar::RadarPacket& packet) {
    while (true) {
      if (mutex.try_lock() && !queue.empty()) {
        packet = queue.front();
        queue.pop();
        queueSize -= 1;
        mutex.unlock();
      }
    }
  };
  void operator>>(radar::RadarPacket& packet) { return retrieve(packet); };

  void closeThread(std::thread* thread) {
    if (thread && thread->joinable()) {
      thread->join();
      thread->~thread();
      delete thread;
      thread = nullptr;
    }
  }
};
}