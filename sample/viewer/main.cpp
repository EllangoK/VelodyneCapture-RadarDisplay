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
std::vector<float> params = { 1.037013, -50.06731667, 30, -11, -8 };

std::queue<std::vector<cv::Vec3f> > radarBufferQueue;
std::vector<cv::Vec3f> radarBuffer;
std::queue<std::vector<cv::Vec3f> > firstObjectQueue;
std::vector<cv::Vec3f> firstObjectBuffer;
std::queue<std::vector<cv::Vec3f> > secondObjectQueue;
std::vector<cv::Vec3f> secondObjectBuffer;
void exposeRadarBuffer()
{
    while (true) {
        if (mutex.try_lock()) {
            radarBuffer.clear();
            firstObjectBuffer.clear();
            secondObjectBuffer.clear();
            if (!radarBufferQueue.empty()) {
                radarBuffer = std::move(radarBufferQueue.front());
                radarBufferQueue.pop();
                if (!firstObjectQueue.empty()) {
                    firstObjectBuffer = std::move(firstObjectQueue.front());
                    firstObjectQueue.pop();
                }
                if (!secondObjectQueue.empty()) {
                    secondObjectBuffer = std::move(secondObjectQueue.front());
                    secondObjectQueue.pop();
                }
            }
            mutex.unlock();
            break;
        }
    }
}
std::deque<radar::RadarPacket> prev;
void generateRadarQueue(radar::RadarServer* radar)
{
    int cycles = 0;
    while (!radar->isEmpty()) {
        radar::RadarPacket data;
        *radar >> data;
        std::vector<cv::Vec3f> temp;
        if (cycles < 10) {
            prev.push_back(data);
        }
        else {
            prev.push_back(data);
            firstObjectQueue.push(data.generateBoundaryFromKernel(prev, 0.02));
            secondObjectQueue.push(data.generateAllPointVec());
            radarBufferQueue.push(data.findBoundary(prev, 20.));
            prev.pop_front();
        }
        cycles++;
    }
}
std::queue<std::vector<cv::Vec3f> > laserBufferQueue;
std::vector<cv::Vec3f> laserBuffer;
void exposeLaserBuffer()
{
    while (true) {
        if (mutex.try_lock()) {
            laserBuffer.clear();
            if (!laserBufferQueue.empty()) {
                laserBuffer = std::move(laserBufferQueue.front());
                laserBufferQueue.pop();
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
    std::system("rm -rf /tmp/radarPacket");
    // Open VelodyneCapture that retrieve from PCAP
    const std::string filename = "../kesselRunVelocity.pcap";
    velodyne::VLP16Capture capture(filename);
    // velodyne::HDL32ECapture capture( filename );

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
    std::vector<cv::Vec3f> localLaserBuffer, localRadarBuffer, lidarPointsInRange,
        firstObject, secondObject;
    while (true) {
        if (!radarServer.isRun()) {
            generateRadarQueue(&radarServer);
            break;
        }
    }
    //do { std::cout << '\n' << "Wait for second"; } while (std::cin.get() != '\n');

    while (!viewer.wasStopped() || true) {
        if (firstRun && !radarServer.isRun()) {
            timedFunction(exposeLaserBuffer, 100);
            usleep(250000);
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
                if (firstObject.size() != firstObjectBuffer.size()) {
                    firstObject = firstObjectBuffer;
                }
                if (secondObject.size() != secondObjectBuffer.size()) {
                    secondObject = secondObjectBuffer;
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
        collection.addCloud(radarCloudMat, cv::viz::Color::green());

        cv::Mat firstObjectMat = cv::Mat(static_cast<int>(firstObject.size()), 1,
            CV_32FC3, &firstObject[0]);
        collection.addCloud(firstObjectMat, cv::viz::Color::red());

        cv::Mat secondObjectMat = cv::Mat(static_cast<int>(secondObject.size()), 1,
            CV_32FC3, &secondObject[0]);
        collection.addCloud(secondObjectMat, cv::viz::Color::black());

        collection.finalize();
        viewer.showWidget("Cloud", collection);
        viewer.spinOnce();
        usleep(10000);
    }

    // Show Point Cloud Collection
    // Close All Viewers
    cv::viz::unregisterAllWindows();
    return 0;
}