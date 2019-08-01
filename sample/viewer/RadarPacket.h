#include <vector>
#include <string>

#include <chrono>

#include <opencv2/opencv.hpp>

namespace radar {
class RadarPacket {
 private:
  long long timeStartSend, timeReceived, timeProcessed;  // microseconds
  std::vector<float> lengthVec;
  std::vector<float> distanceVec;
  int boundaryIndex;
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

  std::vector<cv::Vec3f> generatePointVec() {
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

  float calculateScaleZ(float distance, bool enabled = true) {
    return enabled ? (74.5 * powf(distance, -1.12)) : 0;
  }
};
}