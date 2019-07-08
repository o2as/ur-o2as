#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <o2as_msgs/pc2depth.h>

bool callback(o2as_msgs::pc2depth::Request& req,
    o2as_msgs::pc2depth::Response& res) {

    // Receive topic coming from depth sensor
    sensor_msgs::PointCloud2 msg =
        *ros::topic::waitForMessage<sensor_msgs::PointCloud2>(req.topic_name);

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg ,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*tmp_pc);

    ros::Time timeNow = ros::Time::now();
    std::string frame = "camera";

    // Create response (sensor_msgs::Image)
    sensor_msgs::Image depth_map;
    depth_map.header.stamp = timeNow;
    depth_map.header.frame_id = frame;
    depth_map.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_map.width = msg.width;
    depth_map.height = msg.height;
    depth_map.step = msg.width * sizeof(float);
    depth_map.data.resize(msg.height * msg.width * sizeof(float));

    // Cast
    float* ptr = (float*)(&depth_map.data[0]);

    for (int i = 0; i < msg.height * msg.width; i++) {
        *ptr = tmp_pc->points.at(i).z;
        ptr++;
    }

//    sensor_msgs::fillImage(
//        depth_map, sensor_msgs::image_encodings::TYPE_32FC4,
//        msg.height, msg.width, msg.width * sizeof(float) * 4,
//        &msg.data[0]
//    );
    res.image = depth_map;

    return true;
}


int main(int argc, char **argv) {
    std::cout << "Starting pc2depth ROS service server ...";
    ros::init(argc, argv, "pc2depth_server");
    ros::NodeHandle nh("~");

    ros::ServiceServer srv = nh.advertiseService("pc2depth_service", callback);
    std::cout << "Ready!!!!" << std::endl;
    ros::spin();
    return 0;
}
