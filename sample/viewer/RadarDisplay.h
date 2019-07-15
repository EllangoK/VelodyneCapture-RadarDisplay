#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>
#include <stdexcept>

namespace radar {

class RadarDisplay {
 protected:
  int sockfd, newsockfd, portno;
  char msgBuffer[59];
  socklen_t clilen;
  struct sockaddr_in serv_addr, cli_addr;

  float offsetX, offsetY, offsetZ;

  std::thread* thread;
  std::vector<cv::Vec3f> buffer;

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
    listen(sockfd, 1);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr*)&cli_addr, &clilen);
    if (newsockfd < 0) {
      error("Connection Accept Error");
    }
    bzero(msgBuffer, 59);
  }
  void threadRadarRead() {
    thread = new std::thread(std::bind(&RadarDisplay::radarRead, this));
  }
  void radarRead() {
    int n;
    while (true) {
      n = read(newsockfd, msgBuffer, 59);
      if (n < 0) error("Socket Read Error");
      n = write(newsockfd, "I got your message", 18);
      if (n < 0) error("Socket Write Error");
    }
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
    float radarX, radarUpperY, radarLowerY, radarZ;
    radarX = static_cast<float>(radarDataVec[1]) + offsetX;
    radarUpperY = static_cast<float>(radarDataVec[0] / 2.) + offsetY * 2;
    radarLowerY = static_cast<float>(-radarDataVec[0] / 2.) + offsetY * 2;
    radarZ = 0.0 - offsetZ;  // unknown for now
    if (radarX != 0.0f && radarUpperY != 0.0f) {
      buffer.clear();
      for (int i = ceil(radarLowerY); i <= floor(radarUpperY); i += 2) {
        buffer.push_back(cv::Vec3f(radarX, i, radarZ));
        buffer.push_back(cv::Vec3f(radarX + 1, i, radarZ));
        buffer.push_back(cv::Vec3f(radarX + 2, i, radarZ));
        buffer.push_back(cv::Vec3f(radarX + 3, i, radarZ));
      }
    } else {
      radarX = std::numeric_limits<float>::quiet_NaN();
      radarUpperY = std::numeric_limits<float>::quiet_NaN();
      radarZ = std::numeric_limits<float>::quiet_NaN();
      buffer.push_back(cv::Vec3f(radarX, radarUpperY, radarZ));
    }
    return buffer;
  }

  void close() {
    if (thread && thread->joinable()) {
      thread->join();
      thread->~thread();
      delete thread;
      thread = nullptr;
    }
  }

  void error(const char* msg) {
    perror(msg);
    exit(1);
  }
};
}
