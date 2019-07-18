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

  float offsetX, offsetY, offsetZ;
  float radarX, radarUpperZ, radarLowerZ, radarY;
  float groundZ = 0, scaleZ = 1, scaleX = 1;
  int cycles = 0;
  std::vector<cv::Vec3f> buffer;
  std::vector<cv::Vec3f> lidarPointsInRange;

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
    std::cout << "measured: " << measured << " actual: " << actual
              << " error: " << fabs((measured - actual) / actual) * 100.
              << std::endl;
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
    // u_int64_t now =
    // std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // std::cout << now << std::endl;
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
    std::vector<double> radarDataVec;
    radarDataVec = extractData();
    radarX = (static_cast<float>(radarDataVec[1]) + offsetX) * scaleX;
    radarUpperZ = (static_cast<float>(radarDataVec[0] / 2.) + offsetZ) * scaleZ;
    radarLowerZ =
        (static_cast<float>(-radarDataVec[0] / 2.) + offsetZ) * scaleZ;
    radarY = 0.0 + offsetY * 2;  // unknown for now
    if (radarX != offsetX && radarUpperZ != offsetZ) {
      buffer.clear();
      for (int i = ceil(radarLowerZ); i <= floor(radarUpperZ); i += 2) {
        buffer.push_back(cv::Vec3f(radarX, radarY, i));
        buffer.push_back(cv::Vec3f(radarX, radarY + 1, i));
        buffer.push_back(cv::Vec3f(radarX, radarY + 2, i));
      }
    } else {
      buffer.clear();
      radarX = std::numeric_limits<float>::quiet_NaN();
      radarUpperZ = std::numeric_limits<float>::quiet_NaN();
      radarY = std::numeric_limits<float>::quiet_NaN();
      buffer.push_back(cv::Vec3f(radarX, radarY, radarUpperZ));
    }
    return buffer;
  }

  void findPointsInRange(std::vector<cv::Vec3f> lidarPoints,
                         float percentTolerance) {
    for (int i = 0; i < lidarPoints.size(); i++) {
      bool preFlag = isnanf(lidarPoints[i][0]) || isnanf(radarX) ||
                     lidarPoints[i][0] < 0.0f;
      if (!preFlag) {
        if (isInPercentTolerance(radarX, lidarPoints[i][0], percentTolerance)) {
          lidarPointsInRange.push_back(lidarPoints[i]);
          std::cout << lidarPoints[i][0] << std::endl;
        }
      }
    }
  }
  float findAvgGroundZ() {
    std::vector<float> groundZVec;
    int count = 0;
    for (int i = 0; i < lidarPointsInRange.size(); i++) {
      groundZVec.push_back(lidarPointsInRange[i][2]);
    }
    float groundZ = 0.0f;
    int groundZSize = groundZVec.size();
    int size = std::min(40, groundZSize);
    for (int i = 0; i < size; i++) {
      int minIndex =
          std::distance(groundZVec.begin(),
                        std::min_element(groundZVec.begin(), groundZVec.end()));
      groundZ += groundZVec[minIndex];
      groundZVec.erase(groundZVec.begin() + minIndex);
    }
    groundZ /= size;
    return groundZ;
  }
  float findAvgCurbHeight() {
    std::vector<float> maxZVec;
    int count = 0;
    for (int i = 0; i < lidarPointsInRange.size(); i++) {
      if (lidarPointsInRange[i][2] <= (groundZ += 30)) {
        maxZVec.push_back(lidarPointsInRange[i][2]);
      }
    }
    float maxZ = 0.0f;
    int maxZVecSize = maxZVec.size();
    int size = std::min(40, maxZVecSize);
    for (int i = 0; i < size; i++) {
      int maxIndex = std::distance(
          maxZVec.begin(), std::min_element(maxZVec.begin(), maxZVec.end()));
      groundZ += maxZVec[maxIndex];
      maxZVec.erase(maxZVec.begin() + maxIndex);
    }
    maxZ /= size;
    return maxZ;
  }
  float findAvgX() {
    float X = 0.0f;
    std::cout << "size: " << lidarPointsInRange.size() << std::endl;
    for (int i = 0; i < lidarPointsInRange.size(); i++) {
      X += lidarPointsInRange[i][0];
    }
    return X / lidarPointsInRange.size();
  }

  void fitToLidar(std::vector<cv::Vec3f> lidarPoints) {
    if (cycles > 99 && cycles < 110) {
      findPointsInRange(lidarPoints, 5);
    } else if (cycles == 110) {
      groundZ = findAvgGroundZ();
      float avgX = findAvgX(), avgCurbHeight = findAvgCurbHeight();
      scaleZ = (avgCurbHeight - groundZ) / (radarUpperZ - radarLowerZ);
      scaleX = avgX / radarX;
      std::cout << "scaleZ: " << scaleZ << " scaleX: " << scaleX << std::endl;
      std::cout << "avgCurbHeight: " << avgCurbHeight << " groundZ: " << groundZ
                << std::endl;
      std::cout << "radarUpperZ: " << radarUpperZ
                << " radarLowerZ: " << radarLowerZ << std::endl;
      std::cout << "avgX: " << avgX << " radarX: " << radarX << std::endl;
    }
    cycles++;
    std::cout << cycles << std::endl;
  }

  void close() {}

  void error(const char* msg) {
    perror(msg);
    exit(1);
  }
};
}