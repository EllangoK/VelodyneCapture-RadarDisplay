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

int main( int argc, char* argv[] )
{
    // Open VelodyneCapture that retrieve from Sensor
    //const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.21" );
    //const unsigned short port = 2368;
    //velodyne::VLP16Capture capture( address, port );
    //velodyne::HDL32ECapture capture( address, port );


    //Open VelodyneCapture that retrieve from PCAP
    const std::string filename = "../test3.pcap";
    velodyne::VLP16Capture capture( filename );
    //velodyne::HDL32ECapture capture( filename );
    
    int portno = 12345;
    //5cm up, 22 right, 33 back for static
    radar::RadarDisplay radar(portno, 22, -33, -5);
    //2cm up, 22 right, 11 back for moving
    //radar::RadarDisplay radar(portno, 22, -22, -2);
    radar.startServer();
    radar.threadRadarRead();

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }
    // Create Viewer
    cv::viz::Viz3d viewer( "Velodyne" );

    // Register Keyboard Callback
    viewer.registerKeyboardCallback(
        []( const cv::viz::KeyboardEvent& event, void* cookie ){
            // Close Viewer
            if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
                static_cast<cv::viz::Viz3d*>( cookie )->close();
            }
        }
        , &viewer
    );
    while( capture.isRun() && !viewer.wasStopped() ){
        usleep(41500);
        // Capture One Rotation Data
        cv::viz::WCloudCollection collection;

        std::vector<cv::Vec3f> radarBuffer;
        radarBuffer = radar.generatePointVec();
        cv::Mat radarCloudMat = cv::Mat(static_cast<int>(radarBuffer.size()), 1, CV_32FC3, &radarBuffer[0]);
        collection.addCloud(radarCloudMat, cv::viz::Color::raspberry());
        
        std::vector<velodyne::Laser> lasers;
        std::vector<cv::Vec3f> laserBuffer(lasers.size());

        capture >> lasers;
        if( lasers.empty() ){
            continue;
        }

        // Convert to 3-dimention Coordinates
        for( const velodyne::Laser& laser : lasers ){
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = laser.azimuth  * CV_PI / 180.0;
            const double vertical = laser.vertical * CV_PI / 180.0;

            float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
            float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
            float z = static_cast<float>( ( distance * std::sin( vertical ) ) );

            if( x == 0.0f && y == 0.0f && z == 0.0f ){
                x = std::numeric_limits<float>::quiet_NaN();
                y = std::numeric_limits<float>::quiet_NaN();
                z = std::numeric_limits<float>::quiet_NaN();
            }
            laserBuffer.push_back( cv::Vec3f( x, y, z ) );
        }
        cv::Mat lidarCloudMat = cv::Mat( static_cast<int>(laserBuffer.size() ), 1, CV_32FC3, &laserBuffer[0] );

        collection.addCloud(lidarCloudMat, cv::viz::Color::white());
        collection.finalize();

        // Show Point Cloud Collection
        viewer.showWidget( "Cloud", collection);
        viewer.spinOnce();
    }

    // Close All Viewers
    cv::viz::unregisterAllWindows();
    radar.close();
    return 0;
}
