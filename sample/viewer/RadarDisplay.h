#include <vector>
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

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <opencv2/opencv.hpp>

namespace radar {

class RadarDisplay {
 protected:
  int sockfd, newsockfd, portno;
  char msgBuffer[59];
  socklen_t clilen;
  struct sockaddr_in serv_addr, cli_addr;

  float offsetX, offsetY, offsetZ, lidarOffsetZ = 0;
  float radarX, radarUpperZ, radarLowerZ, radarY;
  float avgCurbHeight = 0, groundZ = 0, scaleZ = 1, scaleX = 1;
  int cycles = 0;
  std::vector<cv::Vec3f> buffer;
  std::vector<cv::Vec3f> lidarPointsInRange, firstObject, secondObject;

 public:
  // Constructor
  RadarDisplay(int port, float X, float Y, float Z) {
    portno = port;
    offsetX = X;
    offsetY = Y;
    offsetZ = Z;
  };
  // Destructor
  ~RadarDisplay() { close(); };

  void timedFunction(std::function<void(void)> func, unsigned int interval) {
    std::thread([func, interval]() {
      while (true) {
        auto x = std::chrono::steady_clock::now() +
                 std::chrono::milliseconds(interval);
        func();
        std::this_thread::sleep_until(x);
      }
    }).detach();
  }
  bool isInPercentTolerance(float measured, float actual,
                            float percentTolerance) {
    return (fabs((measured - actual) / actual) * 100. <= percentTolerance);
  }

  void startServer() {
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
    bzero(msgBuffer, 59);
  }
  void threadRadarRead() {
    timedFunction(std::bind(&RadarDisplay::radarRead, this), 20);
  }
  void radarRead() {
    if (read(newsockfd, msgBuffer, 59) < 0) error("Socket Read Error");
    if (write(newsockfd, "I got your message", 18) < 0)
      error("Socket Write Error");
    buffer.clear();
  }

  std::vector<double> extractData() {
    std::string radarData;
    radarData = std::string(msgBuffer);
    double lengthRdr, widthRdr, distanceRdr, angleRdr;
    // * 100 to convert to centimeters
    try {
      lengthRdr = atof(radarData.substr(0, 14).c_str()) * 100.;
      widthRdr = atof(radarData.substr(15, 29).c_str()) * 100.;
      distanceRdr = atof(radarData.substr(30, 44).c_str()) * 100.;
      angleRdr = atof(radarData.substr(45, 59).c_str()) * 100.;
    } catch (const std::out_of_range& oor) {
      lengthRdr = 0.;
      distanceRdr = 0.;
    }
    std::vector<double> radarDataVec;
    radarDataVec.push_back(lengthRdr);
    radarDataVec.push_back(distanceRdr);
    return radarDataVec;
  }
  std::vector<cv::Vec3f> generatePointVec() {
    calculateScaleZ();
    std::vector<double> radarDataVec;
    radarDataVec = extractData();
    radarX = (static_cast<float>(radarDataVec[1]) + offsetX) * scaleX;
    radarUpperZ =
        (static_cast<float>(radarDataVec[0] / 2.) + offsetZ) * scaleZ +
        lidarOffsetZ;
    /* radarLowerZ =
        (static_cast<float>(-radarDataVec[0] / 2.) + offsetZ) * scaleZ +
        lidarOffsetZ; Deprecated */
    radarLowerZ = groundZ;
    radarY = 0.0 + offsetY;  // unknown for now
    if (radarX != offsetX && radarUpperZ != offsetZ) {
      buffer.clear();
      for (int i = ceil(radarLowerZ); i <= floor(radarUpperZ); i += 2) {
        buffer.push_back(cv::Vec3f(radarX, radarY, i));
        buffer.push_back(cv::Vec3f(radarX, radarY + 1, i));
        buffer.push_back(cv::Vec3f(radarX, radarY + 2, i));
      }
    } else {
      radarX = std::numeric_limits<float>::quiet_NaN();
      radarUpperZ = std::numeric_limits<float>::quiet_NaN();
      radarY = std::numeric_limits<float>::quiet_NaN();
      buffer.push_back(cv::Vec3f(radarX, radarY, radarUpperZ));
    }
    return buffer;
  }

  void fitToLidar(std::vector<cv::Vec3f> lidarPoints) {
    if (cycles > 189 && cycles < 210) {
      findPointsInRange(lidarPoints, 1.0);
    } else if (cycles == 210) {
      groundZ = findGlobalMinAvgZ(lidarPoints, 50, findMinAvgZ(50));
      avgCurbHeight = findMaxAvgZ(100);
      float avgMinX = findMinAvgX(100);
      scaleZ *= (avgCurbHeight - groundZ) / (radarUpperZ - radarLowerZ);
      scaleX *= avgMinX / radarX;
      lidarOffsetZ = groundZ - scaleZ * radarLowerZ;
      std::cout << "scaleZ: " << scaleZ << " scaleX: " << scaleX
                << " avgMinX: " << avgMinX << std::endl;
      std::cout << "radarUpperZ: " << radarUpperZ
                << " radarLowerZ: " << radarLowerZ << " radarX: " << radarX
                << std::endl;
      std::cout << "groundZ: " << groundZ << " curbHeight: " << avgCurbHeight
                << " lidarOffsetZ: " << lidarOffsetZ << std::endl;
    }
    cycles++;
    std::cout << cycles << std::endl;
  }

  std::vector<cv::Vec3f> returnLidarPointsInRange() {
    return lidarPointsInRange;
  }
  void findPointsInRange(std::vector<cv::Vec3f>& lidarPoints,
                         float percentTolerance) {
    for (int i = 0; i < lidarPoints.size(); i++) {
      bool preFlag = isnanf(lidarPoints[i][0]) || isnanf(radarX) ||
                     isnanf(radarUpperZ) || lidarPoints[i][0] < 0.0f;
      if (!preFlag) {
        if (isInPercentTolerance(radarX, lidarPoints[i][0], percentTolerance)) {
          lidarPointsInRange.push_back(lidarPoints[i]);
        }
      }
    }
  }

  float findGlobalMinAvgZ(std::vector<cv::Vec3f>& lidarPoints, int numsamples,
                          float localGroundZ) {
    int size = 0;
    float globalMin = 0;
    for (int i = 0; i < lidarPoints.size(); i++) {
      bool preFlag = isnanf(lidarPoints[i][0]) || isnanf(radarX) ||
                     isnanf(radarUpperZ) || lidarPoints[i][0] < 0.0f;
      if (!preFlag && (lidarPoints[i][2] <= localGroundZ)) {
        globalMin += lidarPoints[i][2];
        size++;
      }
    }
    return globalMin / static_cast<float>(size);
  }
  float findMinAvgZ(int numSamples) {
    std::vector<float> minZVec;
    int count = 0;
    for (int i = 0; i < lidarPointsInRange.size(); i++) {
      minZVec.push_back(lidarPointsInRange[i][2]);
    }
    float minZ = 0.0f;
    int minZSize = minZVec.size();
    int size = std::min(numSamples, minZSize);
    for (int i = 0; i < size; i++) {
      int minIndex = std::distance(
          minZVec.begin(), std::min_element(minZVec.begin(), minZVec.end()));
      minZ += minZVec[minIndex];
      minZVec.erase(minZVec.begin() + minIndex);
    }
    return (minZ / static_cast<float>(size));
  }
  float findMaxAvgZ(int numSamples) {
    std::vector<float> maxZVec;
    int count = 0;
    for (int i = 0; i < lidarPointsInRange.size(); i++) {
      if ((lidarPointsInRange[i][2] <= (groundZ + 30)) &&
          (lidarPointsInRange[i][2] >= groundZ)) {
        maxZVec.push_back(lidarPointsInRange[i][2]);
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
  float findMinAvgX(int numSamples) {
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
  void calculateScaleZ() {
    if (!isnanf(radarX)) {
      scaleZ = 0.726 + -0.437 * logf((radarX - 30) / 100.);
    }
  }

  void fitToLidarY(std::vector<cv::Vec3f> lidarPoints) {
    float firstObjectInitialY, secondObjectInitialY, secondObjectFinalY,
        distanceTraveled;
    if ((cycles > 189 && cycles < 210) || (cycles > 389 && cycles < 410)) {
      findPointsInRange(lidarPoints, 1.0);
    } else if (cycles == 210) {
      firstObjectInitialY =
          findFirstObjectY(1.0, -10);  // known height in lidar scale
      secondObjectInitialY =
          findSecondObjectY(1.0, 10);  // known height in lidar scale
      lidarPointsInRange.clear();
    } else if (cycles == 410) {
      secondObjectFinalY = findSecondObjectY(1.0, 10);
      offsetY =
          (secondObjectFinalY - secondObjectInitialY) - firstObjectInitialY;
      std::cout << "offsetY" << offsetY << std::endl;
    }
    cycles++;
    std::cout << cycles << std::endl;
  }
  std::vector<cv::Vec3f> returnFirstObject() { return firstObject; }
  float findFirstObjectY(float percentTolerance, float height) {
    float y = 0;
    int cycles = 0;
    for (int i = 0; i < lidarPointsInRange.size(); i++) {
      bool preFlag = isnanf(lidarPointsInRange[i][1]) || isnanf(radarX) ||
                     isnanf(radarUpperZ);
      if (!preFlag) {
        if (isInPercentTolerance(height, lidarPointsInRange[i][1],
                                 percentTolerance)) {
          firstObject.push_back(lidarPointsInRange[i]);
          y += lidarPointsInRange[i][1];
          cycles++;
        }
      }
    }
    return y / static_cast<float>(cycles);
  }
  std::vector<cv::Vec3f> returnSecondObject() { return secondObject; }
  float findSecondObjectY(float percentTolerance, float height) {
    float y = 0;
    int cycles = 0;
    for (int i = 0; i < lidarPointsInRange.size(); i++) {
      bool preFlag = isnanf(lidarPointsInRange[i][1]) || isnanf(radarX) ||
                     isnanf(radarUpperZ);
      if (!preFlag) {
        if (isInPercentTolerance(height, lidarPointsInRange[i][1],
                                 percentTolerance)) {
          secondObject.push_back(lidarPointsInRange[i]);
          y += lidarPointsInRange[i][1];
          cycles++;
        }
      }
    }
    return y / static_cast<float>(cycles);
  }

  void close() {}

  void error(const char* msg) {
    perror(msg);
    exit(1);
  }
};
}