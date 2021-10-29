
#define WINVER 0x0A00
#define _WIN32_WINNT 0x0A00

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

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
namespace publish_velodyne
{

typedef pcl::PointXYZI PointType;
using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;


class PublishVelodyne : public nodelet::Nodelet
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    pcl::PointCloud<PointType>::Ptr msg; 

    std::string ipaddress;
    std::string port;
    std::string pcap;
    pcl::PointCloud<PointType>::ConstPtr cloud;
    boost::shared_ptr<pcl::VLPGrabber> grabber;
    boost::signals2::connection connection;
    
    boost::mutex mutex;
    ros::Timer timer;

public:
    PublishVelodyne() = default;
    virtual void onInit()
    {
        ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();

        ROS_INFO("\033[1;32m---->\033[0m Publish Velodyne Started.");
        msg.reset(new pcl::PointCloud<PointType>());
        pub = nh.advertise<pcl::PointCloud<PointType>> ("velodyne_points", 10, this);
        boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> cb = boost::bind(&PublishVelodyne::vlpCallback, this, _1);
   
        nhp.param<std::string>("ipaddress", ipaddress, "192.168.1.201");
        nhp.param<std::string>("port", port, "2369");

        grabber = boost::shared_ptr<pcl::VLPGrabber>(new pcl::VLPGrabber(boost::asio::ip::address::from_string(ipaddress), boost::lexical_cast<unsigned short>(port)));
   
        // Register Callback Function
        connection = grabber->registerCallback(cb);

        // Start Grabber
        grabber->start();
        timer = nh.createTimer(ros::Duration(0.1), boost::bind(&PublishVelodyne::publishTopic, this, _1));
    }

    //Callback
    void vlpCallback(const pcl::PointCloud<PointType>::ConstPtr& cloudPtr)
    {
        // Lock and Copy
        boost::mutex::scoped_lock lock(mutex);
        cloud = cloudPtr;
    }
    void publishTopic(const ros::TimerEvent& event)
    {
        boost::mutex::scoped_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            pcl::copyPointCloud(*cloud, *msg);
            pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            msg->header.frame_id = "velodyne";
            pub.publish(msg);
        }
    }
    ~PublishVelodyne()
    {
        // Stop Grabber
        grabber->stop();

        // Disconnect Callback Function
        if( connection.connected() ){
            connection.disconnect();
        }
    }
};

}PLUGINLIB_EXPORT_CLASS(publish_velodyne::PublishVelodyne, nodelet::Nodelet)
