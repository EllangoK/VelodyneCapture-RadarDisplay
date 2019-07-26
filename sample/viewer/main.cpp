#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/viz/widgets.hpp>

// Include VelodyneCapture Header
#include "VelodyneCapture.h"
#include "RadarDisplay.h"

#include <thread>
#include <future>
#include <unistd.h>
#include <X11/Xlib.h>

cv::Vec3f yaw(cv::Vec3f point, float a)
{
    float x = cosf(a) * point[0] + -sinf(a) * point[1];
    float y = sinf(a) * point[0] + cosf(a) * point[1];
    return cv::Vec3f(x, y, point[2]);
}
cv::Vec3f pitch(cv::Vec3f point, float b)
{
    float x = cosf(b) * point[0] + sinf(b) * point[2];
    float z = -sinf(b) * point[0] + cosf(b) * point[2];
    return cv::Vec3f(x, point[1], z);
}
cv::Vec3f roll(cv::Vec3f point, float c)
{
    float y = cosf(c) * point[1] + -sinf(c) * point[2];
    float z = sinf(c) * point[1] + cosf(c) * point[2];
    return cv::Vec3f(point[0], y, z);
}
cv::Vec3f rotateEulerAngles(cv::Vec3f point, float a, float b, float c)
{
    return roll(pitch(yaw(point, a), b), c);
}
void timedFunction(std::function<void(void)> func, unsigned int interval)
{
    std::thread([func, interval]() {
        while (true) {
            auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);
            func();
            std::this_thread::sleep_until(x);
        }
    }).detach();
}

std::vector<cv::Vec3f> radarBuffer;
std::atomic_int radarCycle = { 0 };
void updateRadarBuffer(radar::RadarDisplay* radar)
{
    while (true) {
        if (*radar >> radarBuffer) {
            radarCycle++;
        }
    }
}
std::atomic_int lidarCycle = { 0 };
std::queue<std::vector<cv::Vec3f> > laserBufferQueue;
std::vector<cv::Vec3f> laserBuffer;
void exposeLaserBuffer()
{
    laserBuffer.clear();
    if (!laserBufferQueue.empty()) {
        laserBuffer = std::move(laserBufferQueue.front());
        lidarCycle++;
        laserBufferQueue.pop();
        std::cout << "lidarCycle: " << lidarCycle << std::endl;
    }
}
void generateLidarQueue(velodyne::VLP16Capture* capture)
{
    while (capture->isRun()) {
        std::vector<cv::Vec3f> laserBuffer;
        std::vector<velodyne::Laser> lasers;
        *capture >> lasers;
        if (lasers.empty()) {
            return;
        }
        for (const velodyne::Laser& laser : lasers) {
            const double distance = static_cast<double>(laser.distance);
            const double azimuth = laser.azimuth * CV_PI / 180.0;
            const double vertical = laser.vertical * CV_PI / 180.0;

            float x = static_cast<float>((distance * std::cos(vertical)) * std::sin(azimuth));
            float y = static_cast<float>((distance * std::cos(vertical)) * std::cos(azimuth));
            float z = static_cast<float>((distance * std::sin(vertical)));

            if (x == 0.0f && y == 0.0f && z == 0.0f) {
                x = std::numeric_limits<float>::quiet_NaN();
                y = std::numeric_limits<float>::quiet_NaN();
                z = std::numeric_limits<float>::quiet_NaN();
            }
            laserBuffer.push_back(rotateEulerAngles(cv::Vec3f(x, y, z), 0 * CV_PI,
                0 * CV_PI, -1. / 11. * CV_PI));
        }
        laserBufferQueue.push(laserBuffer);
    }
}

int main(int argc, char* argv[])
{
    // Open VelodyneCapture that retrieve from Sensor
    // const boost::asio::ip::address address =
    // boost::asio::ip::address::from_string( "192.168.1.21" );
    // const unsigned short port = 2368;
    // velodyne::VLP16Capture capture( address, port );
    // velodyne::HDL32ECapture capture( address, port );

    // Open VelodyneCapture that retrieve from PCAP
    const std::string filename = "../300cmTilt.pcap";
    velodyne::VLP16Capture capture(filename);
    // velodyne::HDL32ECapture capture( filename );

    int portno = 12342;
    radar::RadarDisplay radar(portno, 24, 11, -8);

    if (!capture.isOpen()) {
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }
    // Create Viewer
    cv::viz::Viz3d viewer("Velodyne");

    // Register Keyboard Callback
    viewer.registerKeyboardCallback(
        [](const cv::viz::KeyboardEvent& event, void* cookie) {
            // Close Viewer
            if (event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
                static_cast<cv::viz::Viz3d*>(cookie)->close();
            }
        },
        &viewer);
    generateLidarQueue(&capture);
    bool firstRun = true;
    std::thread t1;
    std::vector<cv::Vec3f> localLaserBuffer, localRadarBuffer;
    int prevCycle = 0;
    while (!viewer.wasStopped() || !radar.isEmpty() || true) {
        if (radar.isQueueBuildOver()) {
            timedFunction(exposeLaserBuffer, 100);
            t1 = std::thread(std::bind(updateRadarBuffer, &radar));
            radar.resetQueueBuild();
        }
        cv::viz::WCloudCollection collection;
        if (localLaserBuffer.size() != laserBuffer.size()) {
            localLaserBuffer = laserBuffer;
        }
        cv::Mat lidarCloudMat = cv::Mat(static_cast<int>(localLaserBuffer.size()),
            1, CV_32FC3, &localLaserBuffer[0]);
        collection.addCloud(lidarCloudMat, cv::viz::Color::white());

        if (localRadarBuffer.size() != radarBuffer.size()) {
            localRadarBuffer = radarBuffer;
        }
        if (lidarCycle != prevCycle) {
            if (!radar.fitToLidar(localLaserBuffer, localRadarBuffer, lidarCycle)) {
                lidarCycle--;
            }
            prevCycle = lidarCycle;
        }
        cv::Mat radarCloudMat = cv::Mat(static_cast<int>(localRadarBuffer.size()),
            1, CV_32FC3, &localRadarBuffer[0]);
        collection.addCloud(radarCloudMat, cv::viz::Color::raspberry());

        collection.finalize();
        viewer.showWidget("Cloud", collection);
        viewer.spinOnce();
    }

    /* radar.fitToLidar(laserBuffer);

std::vector<cv::Vec3f> selectedPoints = radar.returnLidarPointsInRange();
cv::Mat selectedPointsMat = cv::Mat( static_cast<int>(selectedPoints.size() ),
1, CV_32FC3, &selectedPoints[0] );
ollection.addCloud(selectedPointsMat, cv::viz::Color::blue());

std::vector<cv::Vec3f> obj = radar.returnFirstObject();
std::vector<cv::Vec3f> obj2 = radar.returnSecondObject();
cv::Mat objMat = cv::Mat( static_cast<int>(obj.size() ), 1, CV_32FC3, &obj[0]
);
cv::Mat obj2Mat = cv::Mat( static_cast<int>(obj2.size() ), 1, CV_32FC3,
&obj2[0] );
collection.addCloud(objMat, cv::viz::Color::green());
collection.addCloud(obj2Mat, cv::viz::Color::purple()); */

    // Show Point Cloud Collection
    // Close All Viewers
    cv::viz::unregisterAllWindows();
    // radar.close();
    return 0;
}