#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <math.h>
#include <thread> 

namespace radar {

class RadarCapture {
 public:
  int sockfd, newsockfd, portno;
  char msgBuffer[59];
  socklen_t clilen;
  struct sockaddr_in serv_addr, cli_addr;
  int n;
  // Constructor
  RadarCapture(int port){
      portno = port;
  };
  // Destructor
  ~RadarCapture() { close(); };
  void startServer() {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        error("Opening Socket Error");
    }
    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
      error("Binding Error");
    }
    listen(sockfd, 5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
    if (newsockfd < 0) {
      error("Connection Accept Error");
    }
    bzero(msgBuffer, 59);
  }
  std::string radarRead() {
    n = read(newsockfd, msgBuffer, 59);
    if (n < 0) error("Socket Read Error");
    n = write(newsockfd, "I got your message", 18);
    if (n < 0) error("Socket Write Error");

    return std::string(msgBuffer);
  }
  std::vector<double> extractData(){
    std::string radarData(radarRead());
    // * 100 to convert to centimeters
    double lengthRdr, widthRdr, distanceRdr, angleRdr;
    lengthRdr = atof(radarData.substr(0,14).c_str()) * 100.;
    widthRdr = atof(radarData.substr(15,29).c_str()) * 100.;
    distanceRdr = atof(radarData.substr(30,44).c_str()) * 100.;
    angleRdr = atof(radarData.substr(45,59).c_str()) * 100.;
    std::vector<double> radarDataVec;
    radarDataVec.push_back(lengthRdr);
    radarDataVec.push_back(distanceRdr);
    return radarDataVec;
  }
  std::vector<cv::Vec3f> generatePointVec() {
    std::vector<cv::Vec3f> buffer;
    std::vector<double> radarDataVec;
    radarDataVec = extractData();
    float radarX, radarUpperY, radarLowerY, radarZ;
    radarX = static_cast<float>(radarDataVec[1])+16;
    radarUpperY = static_cast<float>(radarDataVec[0]/2.)-66;
    radarLowerY = static_cast<float>(-radarDataVec[0]/2.)-66;
    radarZ = 0.0 - 5; //unknown for now
    //5cm up, 16 right, 33 forward
    if( radarX != 0.0f && radarUpperY != 0.0f) {
      for(int i = ceil(radarLowerY); i <= floor(radarUpperY); i += 2) {
        buffer.push_back(cv::Vec3f(radarX,i,radarZ));
        buffer.push_back(cv::Vec3f(radarX+1,i,radarZ));
        buffer.push_back(cv::Vec3f(radarX+2,i,radarZ));
        buffer.push_back(cv::Vec3f(radarX+3,i,radarZ));
      }
    } else {
        radarX = std::numeric_limits<float>::quiet_NaN();
        radarUpperY = std::numeric_limits<float>::quiet_NaN();
        radarZ = std::numeric_limits<float>::quiet_NaN();
        buffer.push_back(cv::Vec3f(radarX,radarUpperY,radarZ));
    }
    return buffer;
  }

  void close() {}

  void error(const char *msg) {
    perror(msg);
    exit(1);
  }
};
}
