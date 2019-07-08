#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "o2as_realsense_camera/RealSenseCameraConfig.h"
#include "o2as_realsense_camera/GetFrame.h"
#include "o2as_realsense_camera/DumpFrame.h"

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace o2as{

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using image_t = sensor_msgs::Image;
using cloud_t = sensor_msgs::PointCloud2;
using cinfo_t = sensor_msgs::CameraInfo;

class RealSenseCamera
{
public:
    RealSenseCamera(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~RealSenseCamera();

protected:

    bool deviceExists(const std::string& serial);

    //###################################################### 
    // conifiguration
    //###################################################### 

    enum config_entry {
        cfg_send_color = 0,
        cfg_send_depth,
        cfg_send_cloud,
        cfg_count
    };

    void setParam(o2as_realsense_camera::RealSenseCameraConfig &config, config_entry param);
    void dynamicReconfigureCallback(o2as_realsense_camera::RealSenseCameraConfig &config, uint32_t level);
    bool configure();

    //###################################################### 
    // camera info
    //###################################################### 

    rs2_intrinsics getCameraIntrinsics(int tag);
    inline rs2_intrinsics getColorCameraIntrinsics() { return getCameraIntrinsics(rs2_stream::RS2_STREAM_COLOR); }
    inline rs2_intrinsics getDepthCameraIntrinsics() { return getCameraIntrinsics(rs2_stream::RS2_STREAM_DEPTH); }
    bool prepareConversion();

    //###################################################### 
    // Activate / Deactivate
    //###################################################### 

    bool activate();
    void setTriggerMode(bool mode);
    void deactivate();

    //###################################################### 
    // Capture / Dump / Publish
    //###################################################### 

    /// Capture
    bool getAlignedFrame(rs2::frame& color_frame, rs2::frame& depth_frame);
    inline void rsFrameToCvMat(rs2::frame& frame, int format, cv::Mat& cv_image);
    inline void getCvColorImage(rs2::frame& frame, cv::Mat& cv_image);
    inline void getCvDepthImage(rs2::frame& frame, cv::Mat& cv_image);
    PointCloud::Ptr getPointCloud(rs2::frame& depth_frame);
    std_msgs::Header getHeader();
    bool getFrame(bool publish = false, 
        image_t* color_image_msg = NULL, 
        image_t* depth_image_msg = NULL, 
        cloud_t* point_cloud_msg = NULL);
    bool getFrameCallback(
        o2as_realsense_camera::GetFrame::Request & req, 
        o2as_realsense_camera::GetFrame::Response & res);
    
    /// Publish
    inline void publishFrame(
        image_t* color_image_msg = NULL, 
        image_t* depth_image_msg = NULL, 
        cloud_t* point_cloud_msg = NULL);
    void publishFrameCallback(ros::TimerEvent const &);
    void publishCameraInfo();
    void publishInfoCallback(ros::TimerEvent const &);

    /// Dump
    float* getPointCloud2(cv::Mat& cv_depth_image);
    bool saveCloudBinary(const char* filename, int width, int height, float *cloud);
    bool dumpFrameCallback(
        o2as_realsense_camera::DumpFrame::Request & req, 
        o2as_realsense_camera::DumpFrame::Response & res);

    //###################################################### 
    // Transform
    //###################################################### 

    struct float3
    {
        float x, y, z;
    };
    struct quaternion
    {
        double x, y, z, w;
    };

    void publish_static_tf(const ros::Time& t, 
        const float3& trans, const quaternion& q, 
        const std::string& from, const std::string& to);
    void publishStaticTransforms();

private:
    /// The node handle
    ros::NodeHandle root_nh_;
    ros::NodeHandle nh_;

    /// Dynamic reconfigure
    dynamic_reconfigure::Server<o2as_realsense_camera::RealSenseCameraConfig> dynamic_reconfigure_server_;
    const uint32_t set_default_dynamic_reconfig_values = 0xffffffff;

    /// Serial id of the RealSense camera.
    std::string serial_number_;
    /// Size of images to be captured.
    int color_width_;
    int color_height_;
    int depth_width_;
    int depth_height_;

    /// Camera configuration
    rs2::config rs_config_;
    /// Camera pipeline object
    rs2::pipeline rs_pipe_;
    /// State flag of the node
    bool active_;

    /// Camera parameters
    double depth_scale_;
    double inv_param_[9];

    /// If true, publishes images with a frequency of 30Hz.
    bool trigger_mode_;
    bool send_color_;
    bool send_depth_;
    bool send_cloud_;
    /// Timer to trigger frame publishing.
    ros::Timer publish_frame_timer_;
    ros::Timer publish_info_timer_;
    /// Object for handling transportation of images.
    image_transport::ImageTransport image_transport_;

    /// The frame in which the image and point clouds are send.
    std::string camera_frame_;
    /// Publish static transform
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    /// Location where the images and point clouds are stored.
    std::string camera_data_path_;
    std::string color_image_filename_;
    std::string depth_image_filename_;
    std::string point_cloud_filename_;

    struct Publishers {
        /// Publisher for publishing color images.
        image_transport::Publisher color_image;
        /// Publisher for publishing depth images.
        image_transport::Publisher depth_image;
        /// Publisher for publishing raw point clouds.
        ros::Publisher point_cloud;
        /// Publisher for camera info.
        ros::Publisher color_camera_info;
        ros::Publisher depth_camera_info;
    } publishers_;

    struct Servers {
        /// Service server for supplying point clouds and images.
        ros::ServiceServer get_frame;
        /// Service server for dumping image and cloud to disk.
        ros::ServiceServer dump_frame;
    } servers_;	
};

} // namespace o2as
