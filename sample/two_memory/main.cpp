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

std::vector<float> paramsL = { 1.037013, -50.06731667, 30, -11, -8 };
std::vector<float> paramsR = { 1.037013, -50.06731667, 30, -11, -8 };
std::mutex mutex;

std::queue<std::vector<cv::Vec3f> > radarBufferQueueL;
std::vector<cv::Vec3f> radarBufferL;
std::queue<std::vector<cv::Vec3f> > firstObjectQueueL;
std::vector<cv::Vec3f> firstObjectBufferL;
std::queue<std::vector<cv::Vec3f> > secondObjectQueueL;
std::vector<cv::Vec3f> secondObjectBufferL;

std::queue<std::vector<cv::Vec3f> > radarBufferQueueR;
std::vector<cv::Vec3f> radarBufferR;
std::queue<std::vector<cv::Vec3f> > firstObjectQueueR;
std::vector<cv::Vec3f> firstObjectBufferR;
std::queue<std::vector<cv::Vec3f> > secondObjectQueueR;
std::vector<cv::Vec3f> secondObjectBufferR;
bool newData = false;
void exposeRadarBuffer(std::vector<cv::Vec3f>& radarBuffer, std::vector<cv::Vec3f>& firstObjectBuffer, std::vector<cv::Vec3f>& secondObjectBuffer, std::queue<std::vector<cv::Vec3f> > radarBufferQueue, std::queue<std::vector<cv::Vec3f> > firstObjectQueue, std::queue<std::vector<cv::Vec3f> > secondObjectQueue)
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
std::deque<radar::RadarPacket> prevL;
std::deque<radar::RadarPacket> prevR;
void generateRadarQueue(radar::RadarServer* radar, std::deque<radar::RadarPacket> prev, std::queue<std::vector<cv::Vec3f> > radarBufferQueue, std::queue<std::vector<cv::Vec3f> > firstObjectQueue, std::queue<std::vector<cv::Vec3f> > secondObjectQueue)
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

boost::interprocess::named_mutex 
        mem_mutex{
            boost::interprocess::open_or_create, 
            "radar_mutex"
        };

typedef boost::interprocess::allocator<cv::Vec3f, boost::interprocess::managed_shared_memory::segment_manager>  ShmemAllocator;

typedef boost::interprocess::vector<cv::Vec3f, ShmemAllocator> radar_shared;

void updateAndWrite(std::vector<cv::Vec3f>& localRadarBuffer, std::vector<cv::Vec3f>& firstObject, std::vector<cv::Vec3f>& secondObject, std::vector<cv::Vec3f>& radarBuffer, std::vector<cv::Vec3f>& firstObjectBuffer, std::vector<cv::Vec3f>& secondObjectBuffer, std::queue<std::vector<cv::Vec3f> >& radarBufferQueue, boost::interprocess::managed_shared_memory segment, radar_shared*& shared) {
    const ShmemAllocator alloc_inst (segment.get_segment_manager());
    if (radarBufferQueue.size() && mutex.try_lock()) {
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
    }
}

int main(int argc, char* argv[])
{
    struct shm_remove 
    {
       shm_remove() { boost::interprocess::shared_memory_object::remove("radar_vector"); }
       ~shm_remove(){ boost::interprocess::shared_memory_object::remove("radar_vector"); }
    } remover;

    boost::interprocess::managed_shared_memory segmentL(boost::interprocess::create_only, "radar_vector", 1048576);
    boost::interprocess::managed_shared_memory segmentR(boost::interprocess::create_only, "radar_vector", 1048576);

    const ShmemAllocator alloc_instL (segmentL.get_segment_manager());
    const ShmemAllocator alloc_instR (segmentR.get_segment_manager());

    boost::interprocess::named_mutex::remove("radar_mutex");
    radar_shared *sharedL = segmentL.construct<radar_shared>("radar_sharedL")(alloc_instL);
    radar_shared *sharedR = segmentR.construct<radar_shared>("radar_sharedR")(alloc_instR);
    sharedL->push_back(cv::Vec3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
    sharedR->push_back(cv::Vec3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
    mem_mutex.lock();

    std::system("rm -rf /tmp/radarPacketL");
    std::string portL = "/tmp/radarPacketL";
    radar::RadarServer radarServerL(portL, paramsL);
    std::system("rm -rf /tmp/radarPacketR");
    std::string portR = "/tmp/radarPacketR";
    radar::RadarServer radarServerR(portR, paramsR);
    
    std::vector<cv::Vec3f> localRadarBufferL, firstObjectL, secondObjectL;
    std::vector<cv::Vec3f> localRadarBufferR, firstObjectR, secondObjectR;
    
    bool firstRun = true;
    while (true) {
        if (firstRun && !radarServerL.isRun() && !radarServerR.isRun()) {
            generateRadarQueue(&radarServerL, prevL, radarBufferQueueL, firstObjectQueueL, secondObjectQueueL);
            generateRadarQueue(&radarServerR, prevR, radarBufferQueueR, firstObjectQueueR, secondObjectQueueR);
            firstObjectL = firstObjectQueueL.front();
            sharedL->assign(firstObjectL.begin(), firstObjectL.end());
            firstObjectR = firstObjectQueueR.front();
            sharedR->assign(firstObjectR.begin(), firstObjectR.end());
            timedFunction(std::bind(exposeRadarBuffer, radarBufferL, firstObjectL, secondObjectL, radarBufferQueueL, firstObjectQueueL, secondObjectQueueL), 250);
            timedFunction(std::bind(exposeRadarBuffer, radarBufferR, firstObjectR, secondObjectR, radarBufferQueueR, firstObjectQueueR, secondObjectQueueR), 250);
            firstRun = false;
            mem_mutex.unlock();
            continue;
        }
        while (true && !firstRun) {
            if (mutex.try_lock()) {
                if (localRadarBufferL.size() != radarBufferL.size()) {
                    localRadarBufferL = radarBufferL;
                }
                if (localRadarBufferR.size() != radarBufferR.size()) {
                    localRadarBufferR = radarBufferR;
                }
                if (secondObjectL.size() != secondObjectBufferL.size() && secondObjectBufferL.size()) {
                    secondObjectL = secondObjectBufferL;
                }
                if (secondObjectR.size() != secondObjectBufferR.size() && secondObjectBufferR.size()) {
                    secondObjectR = secondObjectBufferR;
                }
                if (firstObjectL != firstObjectBufferR && firstObjectBufferR.size()) {
                    firstObjectL = firstObjectBufferR;
                    mem_mutex.lock();
                    segmentL.destroy<radar_shared>("radar_sharedL");
                    radar_shared *sharedL = segmentL.construct<radar_shared>("radar_sharedL")(alloc_instL);
                    sharedR->assign(firstObjectL.begin(), firstObjectL.end());
                    std::cout << sharedL[0][0] << std::endl;
                }
                if (firstObjectR != firstObjectBufferR && firstObjectBufferR.size()) {
                    firstObjectR = firstObjectBufferR;
                    segmentR.destroy<radar_shared>("radar_sharedR");
                    radar_shared *sharedR = segmentR.construct<radar_shared>("radar_sharedR")(alloc_instR);
                    sharedR->assign(firstObjectR.begin(), firstObjectR.end());
                    std::cout << sharedR[0][0] << std::endl;
                    mem_mutex.unlock();
                }
                mutex.unlock();
            }
        }
       usleep(10000);
    }
    return 0;
}