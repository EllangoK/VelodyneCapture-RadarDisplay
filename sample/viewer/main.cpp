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
    //5cm up, 16 right, 33 back
    radar::RadarDisplay radar(portno, 16, -33, -5);
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
        usleep(40000);
        // Capture One Rotation Data
        cv::viz::WCloudCollection collection();
        std::vector<velodyne::Laser> lasers;
        std::vector<cv::Vec3f> buffer;
        buffer = radar.generatePointVec();

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
            buffer.push_back( cv::Vec3f( x, y, z ) );
        }

        // Create Widget
        cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_32FC3, &buffer[0] );
        cv::viz::WCloud cloud( cloudMat );
        // Show Point Cloud
        viewer.showWidget( "Cloud", cloud);
        viewer.spinOnce();
    }

    // Close All Viewers
    cv::viz::unregisterAllWindows();
    radar.close();
    return 0;
}
