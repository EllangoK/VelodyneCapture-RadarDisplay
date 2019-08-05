#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/viz/widgets.hpp>

#include "VelodyneCapture.h"
#include "RadarDisplay.h"
#include "RadarControl.h"

#include <thread>
#include <future>
#include <unistd.h>

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
std::mutex mutex;

std::atomic_int radarCycle = { 0 };
std::queue<std::vector<cv::Vec3f> > radarBufferQueue;
std::vector<cv::Vec3f> radarBuffer;
void exposeRadarBuffer()
{
    while (true) {
        if (mutex.try_lock()) {
            radarBuffer.clear();
            if (!radarBufferQueue.empty()) {
                radarBuffer = std::move(radarBufferQueue.front());
                radarCycle++;
                radarBufferQueue.pop();
                std::cout << "radarCycle: " << radarCycle << std::endl;
            }
            mutex.unlock();
            break;
        }
    }
}
void generateRadarQueue(radar::RadarServer* radar)
{
    while (!radar->isEmpty()) {
        radar::RadarPacket data;
        *radar >> data;
        radarBufferQueue.push(data.generateAllPointVec());
    }
}
std::atomic_int lidarCycle = { 0 };
std::queue<std::vector<cv::Vec3f> > laserBufferQueue;
std::vector<cv::Vec3f> laserBuffer;
void exposeLaserBuffer()
{
    while (true) {
        if (mutex.try_lock()) {
            laserBuffer.clear();
            if (!laserBufferQueue.empty()) {
                laserBuffer = std::move(laserBufferQueue.front());
                lidarCycle++;
                laserBufferQueue.pop();
                std::cout << "lidarCycle: " << lidarCycle << std::endl;
            }
            mutex.unlock();
            break;
        }
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
    const std::string filename = "../" + std::string(argv[1]) + ".pcap";
    velodyne::VLP16Capture capture(filename);
    // velodyne::HDL32ECapture capture( filename );

    std::vector<float> params = { 0.964308, -50.06731667, 30, -11, -8 };
    std::string port = "/tmp/radarPacket";
    radar::RadarServer radarServer(port, params);

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
    int prevCycle = 0;
    std::vector<cv::Vec3f> localLaserBuffer, localRadarBuffer, lidarPointsInRange, firstObject, secondObject;

    while (!viewer.wasStopped() || true) {
        if (firstRun && !radarServer.isRun()) {
            generateRadarQueue(&radarServer);
            timedFunction(exposeLaserBuffer, 100);
            timedFunction(exposeRadarBuffer, 250);
            firstRun = false;
        }

        cv::viz::WCloudCollection collection;
        while (true) {
            if (mutex.try_lock()) {
                if (localLaserBuffer.size() != laserBuffer.size()) {
                    localLaserBuffer = laserBuffer;
                }
                if (localRadarBuffer.size() != radarBuffer.size()) {
                    localRadarBuffer = radarBuffer;
                }
                mutex.unlock();
                break;
            }
        }

        cv::Mat lidarCloudMat = cv::Mat(static_cast<int>(localLaserBuffer.size()),
            1, CV_32FC3, &localLaserBuffer[0]);
        collection.addCloud(lidarCloudMat, cv::viz::Color::white());

        cv::Mat radarCloudMat = cv::Mat(static_cast<int>(localRadarBuffer.size()),
            1, CV_32FC3, &localRadarBuffer[0]);
        collection.addCloud(radarCloudMat, cv::viz::Color::orange_red());

        collection.finalize();
        viewer.showWidget("Cloud", collection);
        viewer.spinOnce();
        usleep(20000);
    }

    // Show Point Cloud Collection
    // Close All Viewers
    cv::viz::unregisterAllWindows();
    return 0;
}