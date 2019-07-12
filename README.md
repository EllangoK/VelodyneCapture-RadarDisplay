VelodyneCapture
===============

VelodyneCapture is the general capture class to retrieve the laser data from Velodyne sensors using Boost.Asio and PCAP.  
VelodyneCapture will be able to retrieve lasers infomation about azimuth, vertical and distance that capture at one rotation or a single data packet.  
This class supports direct capture form Velodyne sensors, or capture from PCAP files.  
( This class only supports VLP-16 and HDL-32E sensor, it does not support Dual Return mode. )  


Environment
-----------
If direct capture from sensors is desired, VelodyneCapture requires Boost.Asio and its dependent libraries ( Boost.System, Boost.Date_Time, Boost.Regex ).  
Please define <code>HAVE_BOOST</code> in preprocessor.  
* Boost.Asio  

If capture from PCAP files is instead desired, VelodyneCapture requires PCAP.  
Please define <code>HAVE_PCAP</code> in preprocessor.  
* libpcap ( or WinPCAP )  


Sample
------
This repository has two sample programs.  
* simple  
  This sample program displays data on standard output.  
  This sample program depends on the following libraries: ( Boost.Asio or libpcap ( or WinPCAP ) ) of VelodyneCapture class.  

* viewer  
  This sample program displays data on a point cloud viewer.  
  This sample program depends on OpenCV Viz module in addition to the above libraries ( Boost.Asio or libpcap ( or WinPCAP ) ) of VelodyneCapture class.  

License
-------
Copyright &copy; 2017 Tsukasa SUGIURA  
Distributed under the [MIT License](http://www.opensource.org/licenses/mit-license.php "MIT License | Open Source Initiative").  

Contact
-------
* Tsukasa Sugiura
    * <t.sugiura0204@gmail.com>
    * <https://twitter.com/UnaNancyOwen>
    * <http://UnaNancyOwen.com>

RadarCapture
===============

RadarCapture is a general display capture class to retrieve radar data from the Texas Instrument mmWave AWR1443 automotive radar sensor. It converts the output of side mounted automotive radar that is oriented to capture a vertical slice, into the point cloud and maps it on to existing LIDAR data.
It relies on contour characteristics to be sent via socket, using length, width, distance and angle.
It also relies on offset from LIDAR to properly map the radar readings into the point cloud.
This class has only been tested with the Texas Instrument mmWave AWR1443, but theoretically should be able to display and radar if the orientation is the same and the correct characteristics have been provided.

Environment
-----------
Uses BSD sockets to transfer information from radar capture source to the display.
Uses OpenCV cv::Vec3f for point cloud value storage.
