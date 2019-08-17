#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/viz/widgets.hpp>

#include "RadarControl.h"

#include <thread>
#include <future>
#include <unistd.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

std::vector<float> params = { 1.037013, -50.06731667, 30, -11, -8 };
std::mutex mutex;
std::queue<std::vector<cv::Vec3f> > radarBufferQueue;
std::vector<cv::Vec3f> radarBuffer;
std::queue<std::vector<cv::Vec3f> > firstObjectQueue;
std::vector<cv::Vec3f> firstObjectBuffer;
std::queue<std::vector<cv::Vec3f> > secondObjectQueue;
std::vector<cv::Vec3f> secondObjectBuffer;
bool newData = false;
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
                newData = true;
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
            radarBufferQueue.push(
                data.generatePointVec(data.getBoundaryIndex()));
            prev.push_back(data);
        }
        else {
            firstObjectQueue.push(data.generateBoundaryFromKernel(prev, 0.02));
            secondObjectQueue.push(data.generateAllPointVec());
            radarBufferQueue.push(data.findBoundary(prev, 20.));
            prev.push_back(data);
            prev.pop_front();
        }
        cycles++;
    }
}

void timedFunction(std::function<void(void)> func, unsigned int interval) {
    std::thread([func, interval]() {
        while (true) {
            auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(interval);
            func();
            std::this_thread::sleep_until(x);
        }
    }).detach();
}

typedef boost::interprocess::allocator<cv::Vec3f, boost::interprocess::managed_shared_memory::segment_manager>  ShmemAllocator;

typedef boost::interprocess::vector<cv::Vec3f, ShmemAllocator> radar_shared;

int main(int argc, char* argv[])
{
    struct shm_remove 
    {
       shm_remove() { boost::interprocess::shared_memory_object::remove("radar_vector"); }
       ~shm_remove(){ boost::interprocess::shared_memory_object::remove("radar_vector"); }
    } remover;

    boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only, "radar_vector", 1048576);

    const ShmemAllocator alloc_inst (segment.get_segment_manager());
    boost::interprocess::named_mutex::remove("radar_mutex");
    radar_shared *shared = segment.construct<radar_shared>("radar_shared")(alloc_inst);
    shared->push_back(cv::Vec3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
    boost::interprocess::named_mutex 
            mem_mutex{
                boost::interprocess::open_or_create, 
                "radar_mutex"
            };
    mem_mutex.lock();
    std::system("rm -rf /tmp/radarPacket");
    std::string port = "/tmp/radarPacket";
    radar::RadarServer radarServer(port, params);
    std::vector<cv::Vec3f> localRadarBuffer, firstObject, secondObject;
    bool firstRun = true;
    while (true) {
        if (firstRun && !radarServer.isRun()) {
            generateRadarQueue(&radarServer);
            timedFunction(exposeRadarBuffer, 250);
            firstObject = firstObjectBuffer;
            shared->assign(firstObject.begin(), firstObject.end());
            firstRun = false;
            mem_mutex.unlock();
            continue;
        }
        while (true && !firstRun && radarBufferQueue.size()) {
            if (mutex.try_lock()) {
                if (localRadarBuffer.size() != radarBuffer.size()) {
                    localRadarBuffer = radarBuffer;
                }
                if (firstObject != firstObjectBuffer && firstObjectBuffer.size()) {
                    firstObject = firstObjectBuffer;
                    mem_mutex.lock();
                    segment.destroy<radar_shared>("radar_shared");
                    radar_shared *shared = segment.construct<radar_shared>("radar_shared")(alloc_inst);
                    shared->assign(firstObject.begin(), firstObject.end());
                    std::cout << shared[0][0] << std::endl;
                    mem_mutex.unlock();
                }
                if (secondObject.size() != secondObjectBuffer.size() && secondObjectBuffer.size()) {
                    secondObject = secondObjectBuffer;
                }
                mutex.unlock();
                break;
            }
        }
       usleep(10000);
    }
    return 0;
}