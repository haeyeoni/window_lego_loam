
#define WINVER 0x0A00
#define _WIN32_WINNT 0x0A00

#include <iostream>
#include <string>
#include <vector>

#include <chrono>
#include <iostream>
#include <time.h>
#include <ctime>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Point Type

typedef pcl::PointXYZ PointType;
using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;


int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "publish_velodyne");
    ROS_INFO("\033[1;32m---->\033[0m Publish Velodyne Started.");

    if( pcl::console::find_switch( argc, argv, "-help" ) ){
        std::cout << "usage: " << argv[0]
            << " [-ipaddress <192.168.1.201>]"
            << " [-port <2368>]"
            << " [-pcap <*.pcap>]"
            << " [-help]"
            << std::endl;
        return 0;
    }

    std::string ipaddress( "192.168.1.201" );
    std::string port( "2369" );
    std::string pcap;

    pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
    pcl::console::parse_argument( argc, argv, "-port", port );
    pcl::console::parse_argument( argc, argv, "-pcap", pcap );

    std::cout << "-ipadress : " << ipaddress << std::endl;
    std::cout << "-port : " << port << std::endl;
    std::cout << "-pcap : " << pcap << std::endl;

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    auto millisec_prev = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto millisec_curr = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();


    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );
            /* Point Cloud Processing */
            cloud = ptr;
        };

    // VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;
    if( !pcap.empty() ){
        std::cout << "Capture from PCAP..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( pcap ) );
    }
    else if( !ipaddress.empty() && !port.empty() ){
        std::cout << "Capture from Sensor..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
    }

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

    // ROS
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("velodyne_points", 1000);
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);

    while (nh.ok())
    {
        boost::mutex::scoped_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            pcl::copyPointCloud(*cloud, *msg);
            pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            msg->header.frame_id = "velodyne";
            pub.publish (msg);
        }
        ros::spinOnce ();
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }

    return 0;
}