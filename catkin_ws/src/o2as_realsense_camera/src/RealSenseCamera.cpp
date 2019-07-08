
#include "o2as_realsense_camera/RealSenseCamera.h"

#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace o2as{

RealSenseCamera::RealSenseCamera(ros::NodeHandle nh, ros::NodeHandle private_nh)
: root_nh_(nh), nh_(private_nh), image_transport_(private_nh), dynamic_reconfigure_server_(private_nh)
{
    ROS_INFO("initialize realsense camera.");

    // Setup dynamic reconfigure
    dynamic_reconfigure::Server<o2as_realsense_camera::RealSenseCameraConfig>::CallbackType f;
    f = boost::bind(&RealSenseCamera::dynamicReconfigureCallback, this, _1, _2);
    dynamic_reconfigure_server_.setCallback(f);
    ROS_INFO("prepare for data conversion.");

    // Initialize camera parameters
    depth_scale_ = 0.0;
    memset(inv_param_, 0, 9 * sizeof(double));
    if (configure() == false) {
        throw "failed to configure realsense camera device.";
    }

    // Activate
    active_ = false;
    if (activate() == false) {
        throw "failed to activate realsense camera device.";
    }
}

RealSenseCamera::~RealSenseCamera()
{
    deactivate();
}

bool RealSenseCamera::deviceExists(const std::string& serial)
{
    rs2::context ctx;    
    for (auto&& dev : ctx.query_devices()) 
    {
        std::string s = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        if (s == serial) {
            return true;
        }
    }
    return false;
}

//###################################################### 
// conifiguration
//###################################################### 

void RealSenseCamera::setParam(o2as_realsense_camera::RealSenseCameraConfig &config, config_entry param)
{
    switch (param) {
    case cfg_send_color:
        ROS_DEBUG_STREAM("send_color: " << config.send_color);
        send_color_ = config.send_color;
        break;
    case cfg_send_depth:
        ROS_DEBUG_STREAM("send_depth: " << config.send_depth);
        send_depth_ = config.send_depth;
        break;
    case cfg_send_cloud:
        ROS_DEBUG_STREAM("send_cloud: " << config.send_cloud);
        send_cloud_ = config.send_cloud;
        break;
    }
}

void RealSenseCamera::dynamicReconfigureCallback(o2as_realsense_camera::RealSenseCameraConfig &config, uint32_t level)
{
    if (set_default_dynamic_reconfig_values == level)
    {
        for (int i = 1; i < cfg_count; ++i)
        {
            ROS_DEBUG_STREAM("config = " << i);
            setParam(config ,(config_entry)i);
        }
    }
    else
    {
        setParam(config, (config_entry)level);
    }
}

/// Load parameters and connect to the camera
bool RealSenseCamera::configure() 
{
    ROS_INFO("get ros parameters.");

    // load ROS parameters
    nh_.param<std::string>("serial_number", serial_number_, "");
    nh_.param<int>("color_width", color_width_, 640);
    nh_.param<int>("color_height", color_height_, 360);
    nh_.param<int>("depth_width", depth_width_, 640);
    nh_.param<int>("depth_height", depth_height_, 480);
    nh_.param<std::string>("camera_frame", camera_frame_, "depth_image_frame");
    nh_.param<bool>("send_color", send_color_, true);
    nh_.param<bool>("send_depth", send_depth_, true);
    nh_.param<bool>("send_cloud", send_cloud_, true);
    nh_.param<bool>("trigger_mode", trigger_mode_, false);
    ROS_INFO_STREAM("check device serial number : " << serial_number_);

    // Enable camera
    if (!deviceExists(serial_number_)) {
        ROS_ERROR("realsense device with serial number %s not exists", serial_number_.c_str());
        return false;
    }
    ROS_INFO("enable device and stream.");
    rs_config_.enable_device(serial_number_);
    rs_config_.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width_, color_height_, rs2_format::RS2_FORMAT_BGR8);
    rs_config_.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width_, depth_height_, rs2_format::RS2_FORMAT_Z16 );
    return true;
}

//###################################################### 
// camera info
//###################################################### 

rs2_intrinsics RealSenseCamera::getCameraIntrinsics(int tag)
{
    rs2::pipeline_profile pipeline_profile = rs_pipe_.get_active_profile();
    rs2::stream_profile color_profile = pipeline_profile.get_stream(rs2_stream::RS2_STREAM_COLOR);
    rs2::video_stream_profile video_stream_profile_color = color_profile.as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics_color = video_stream_profile_color.get_intrinsics();
    return intrinsics_color;
}

void invMatrix3x3(float src[9], double dst[9])
{
    double det = (double)src[0] * src[4] * src[8] + src[3] * src[7] * src[2] + src[6] * src[1] * src[5] - src[0] * src[7] * src[5] - src[6] * src[4] * src[2] - src[3] * src[1] * src[8];
    if (fabs(det) < 1e-6) {
        memset(dst, 0, 9 * sizeof(double));
    }
    else {
        double inv_det = 1.0 / det;
        dst[0] = ((double)src[4] * src[8] - src[5] * src[7]) * inv_det;
        dst[1] = ((double)src[2] * src[7] - src[1] * src[8]) * inv_det;
        dst[2] = ((double)src[1] * src[5] - src[2] * src[4]) * inv_det;
        dst[3] = ((double)src[5] * src[6] - src[3] * src[8]) * inv_det;
        dst[4] = ((double)src[0] * src[8] - src[2] * src[6]) * inv_det;
        dst[5] = ((double)src[2] * src[3] - src[0] * src[5]) * inv_det;
        dst[6] = ((double)src[3] * src[7] - src[4] * src[6]) * inv_det;
        dst[7] = ((double)src[1] * src[6] - src[0] * src[7]) * inv_det;
        dst[8] = ((double)src[0] * src[4] - src[1] * src[3]) * inv_det;
    }
}

bool RealSenseCamera::prepareConversion()
{
    rs2::pipeline_profile pipeline_profile = rs_pipe_.get_active_profile();

    // depth_scale
    rs2::depth_sensor depth_sensor = pipeline_profile.get_device().first<rs2::depth_sensor>();
    depth_scale_ = (double)depth_sensor.get_depth_scale();

    // inv_matrix
    rs2_intrinsics intrinsics_color = getColorCameraIntrinsics();
    float cam_param[9] = {intrinsics_color.fx, 0.0, intrinsics_color.ppx, 0.0, intrinsics_color.fy, intrinsics_color.ppy, 0.0, 0.0, 1.0};
    invMatrix3x3(cam_param, inv_param_);
}

//###################################################### 
// Activate / Deactivate
//###################################################### 

/// Activate ROS service servers and publishers
bool RealSenseCamera::activate()
{
    // start pipeline
    rs2::pipeline_profile pipeline_profile = rs_pipe_.start(rs_config_);
    prepareConversion();

    // activate service servers
    servers_.get_frame = nh_.advertiseService("get_frame", &RealSenseCamera::getFrameCallback, this);
    servers_.dump_frame = nh_.advertiseService("dump_frame", &RealSenseCamera::dumpFrameCallback, this);

    // activate publishers
    publishers_.color_image = image_transport_.advertise("color/image_raw", 1, true);
    publishers_.depth_image = image_transport_.advertise("depth/image_raw", 1, true);
    publishers_.point_cloud = nh_.advertise<PointCloud>("cloud", 1, true);
    publishers_.color_camera_info = nh_.advertise<cinfo_t>("color/camera_info", 1, true);
    publishers_.depth_camera_info = nh_.advertise<cinfo_t>("depth/camera_info", 1, true);

    // start image publishing timer
    publish_frame_timer_ = nh_.createTimer(ros::Rate(30), &RealSenseCamera::publishFrameCallback, this);
    publish_info_timer_ = nh_.createTimer(ros::Rate(30), &RealSenseCamera::publishInfoCallback, this);
    publish_info_timer_.start();
    setTriggerMode(trigger_mode_);
    active_ = true;
    return true;
}
void RealSenseCamera::setTriggerMode(bool mode)
{
    if (trigger_mode_ == false) {
        publish_frame_timer_.start();
    } else {
        publish_frame_timer_.stop();
    }
}
void RealSenseCamera::deactivate() 
{
    if (active_ == false) {
        return;
    }
    active_ = false;

    // Deactivate service servers
    servers_.get_frame.shutdown();
    servers_.dump_frame.shutdown();

    // Deactivate publishers
    publishers_.color_image.shutdown();
    publishers_.depth_image.shutdown();
    publishers_.point_cloud.shutdown();

    // Deactivate timers
    publish_frame_timer_.stop();

    // Release the camera
    rs_pipe_.stop();
}

//###################################################### 
// Capture / Dump / Publish
//###################################################### 

bool RealSenseCamera::getAlignedFrame(rs2::frame& color_frame, rs2::frame& depth_frame)
{
    try 
    {
        rs2::frameset frameset = rs_pipe_.wait_for_frames();
        rs2::align align(rs2_stream::RS2_STREAM_COLOR);
        rs2::frameset aligned_frameset = align.process(frameset);
        if (!aligned_frameset.size()) {
            return false;
        }
        color_frame = aligned_frameset.get_color_frame();
        depth_frame = aligned_frameset.get_depth_frame();
    }
    catch (const char *error_message) {
        ROS_ERROR("%s", error_message);
        return false;
    }
    return true;
}

inline void RealSenseCamera::rsFrameToCvMat(rs2::frame& frame, int format, cv::Mat& cv_image)
{
    int width  = frame.as<rs2::video_frame>().get_width();
    int height = frame.as<rs2::video_frame>().get_height();
    cv_image = cv::Mat(height, width, format, const_cast<void*>(frame.get_data()));
}

inline void RealSenseCamera::getCvColorImage(rs2::frame& frame, cv::Mat& cv_image)
{
    rsFrameToCvMat(frame, CV_8UC3, cv_image);
}

inline void RealSenseCamera::getCvDepthImage(rs2::frame& frame, cv::Mat& cv_image)
{
    rsFrameToCvMat(frame, CV_16SC1, cv_image);
}

/// conversion from depth image to point cloud using realsense2 sdk
PointCloud::Ptr RealSenseCamera::getPointCloud(rs2::frame& depth_frame)
{
    rs2::pointcloud pc;
    rs2::points points = pc.calculate(depth_frame);
    PointCloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    return cloud;
}

std_msgs::Header RealSenseCamera::getHeader()
{
    std_msgs::Header header;
    header.frame_id = camera_frame_;
    header.stamp    = ros::Time::now();
    return header;		
}

bool RealSenseCamera::getFrame(bool publish, 
    image_t* color_image_msg, 
    image_t* depth_image_msg, 
    cloud_t* point_cloud_msg)
{
    ROS_ASSERT(active_ == true);
    ROS_DEBUG_STREAM("getFrame. "
        << "publish: "      << publish       << ", "
        << "trigger_mode: " << trigger_mode_ << ", "
        << "send_color: "   << send_color_   << ", "
        << "send_depth: "   << send_depth_   << ", "
        << "send_cloud: "   << send_cloud_
        );

    // get frame
    rs2::frame color_frame, depth_frame;
    getAlignedFrame(color_frame, depth_frame);
    
    // prepare message
    std_msgs::Header header = getHeader();
    cv_bridge::CvImage cv_color(header, sensor_msgs::image_encodings::BGR8, cv::Mat());
    cv_bridge::CvImage cv_depth(header, sensor_msgs::image_encodings::MONO16, cv::Mat());
    
    // get cv image
    getCvColorImage(color_frame, cv_color.image);
    getCvDepthImage(depth_frame, cv_depth.image);
    PointCloud::Ptr point_cloud = getPointCloud(depth_frame);
    
    // to ros message
    if (color_image_msg!=NULL && send_color_==true) {
        *color_image_msg = *cv_color.toImageMsg();
    }
    if (color_image_msg!=NULL && send_depth_==true) {
        *depth_image_msg = *cv_depth.toImageMsg();
    }
    if (color_image_msg!=NULL && send_cloud_==true) {
        pcl::toROSMsg(*point_cloud, *point_cloud_msg);
    }
    
    // publish point cloud if requested
    if (publish) {
        publishFrame(color_image_msg, depth_image_msg, point_cloud_msg);
    }
    return true;
}

bool RealSenseCamera::getFrameCallback(
    o2as_realsense_camera::GetFrame::Request & req, 
    o2as_realsense_camera::GetFrame::Response & res) 
{
    return getFrame(req.publish, &res.color_image, &res.depth_image, &res.point_cloud);
}

inline void RealSenseCamera::publishFrame(
    image_t* color_image_msg, 
    image_t* depth_image_msg, 
    cloud_t* point_cloud_msg) 
{
    if (color_image_msg) {
        publishers_.color_image.publish(*color_image_msg);
    }
    if (depth_image_msg) {
        publishers_.depth_image.publish(*depth_image_msg);
    }
    if (point_cloud_msg) {
        publishers_.point_cloud.publish(*point_cloud_msg);
    }
}

void RealSenseCamera::publishFrameCallback(ros::TimerEvent const &) 
{
    ROS_DEBUG("publish frame callback");
    image_t color_image_msg;
    image_t depth_image_msg; 
    cloud_t point_cloud_msg;
    getFrame(true, &color_image_msg, &depth_image_msg, &point_cloud_msg);
}

/// Conversion from depth image to point cloud using camera parameter
float* RealSenseCamera::getPointCloud2(cv::Mat& cv_depth_image)
{
    int width  = cv_depth_image.cols;
    int height = cv_depth_image.rows;
    float *cloud = (float*)malloc(width * height * 3 * sizeof(float));
    if (cloud == NULL) {
        ROS_ERROR_STREAM("get point cloud : memory allocation error");
        return NULL;
    }

    int pos = 0;
    double proj_x, proj_y, depth;
    const float max_depth = 1e+6f + 1.0f;
    unsigned short *ptr_depth;
    double scale = depth_scale_ * 1000; // in mm unit
    for (int y = 0; y < height; y++) {
        ptr_depth = cv_depth_image.ptr<unsigned short>(y);
        for (int x = 0; x < width; x++) {
            proj_x = inv_param_[0] * (double)x + inv_param_[1] * (double)y + inv_param_[2];
            proj_y = inv_param_[3] * (double)x + inv_param_[4] * (double)y + inv_param_[5];
            depth  = scale  * (double)ptr_depth[x];
            cloud[pos  ] = (float)(proj_x * depth); 
            cloud[pos+1] = (float)(proj_y * depth);
            cloud[pos+2] = (float)depth;
            pos += 3;
        }
    }
    return cloud;
}
bool RealSenseCamera::saveCloudBinary(const char* filename, int width, int height, float *cloud)
{
    FILE *fout = fopen(filename, "wb");
    if (fout == NULL) {
        ROS_ERROR_STREAM("cannot open point cloud file - " << filename);
        return false;
    }
    fwrite(cloud, sizeof(float), width * height * 3, fout);
    fclose(fout);
    return true;
}

bool RealSenseCamera::dumpFrameCallback(
    o2as_realsense_camera::DumpFrame::Request & req, 
    o2as_realsense_camera::DumpFrame::Response & res) 
{
    // get frame
    rs2::frame color_frame, depth_frame;
    getAlignedFrame(color_frame, depth_frame);

    // save color image
    if (!req.color_image_filename.empty()) {
        cv::Mat cv_color_image;
        getCvColorImage(color_frame, cv_color_image);
        cv::imwrite((char*)req.color_image_filename.c_str(), cv_color_image);
    }

    // save depth image and point cloud
    if (!req.depth_image_filename.empty() || !req.point_cloud_filename.empty()) {
        cv::Mat cv_depth_image;
        getCvDepthImage(depth_frame, cv_depth_image);
        if (!req.depth_image_filename.empty()) {
            cv::imwrite((char*)req.depth_image_filename.c_str(), cv_depth_image);
        }
        if (!req.point_cloud_filename.empty()) {
            float* cloud = getPointCloud2(cv_depth_image);
            saveCloudBinary((char*)req.point_cloud_filename.c_str(), cv_depth_image.cols, cv_depth_image.rows, cloud);
            free(cloud);
        }
    }
    return true;
}

void toRosCameraInfo(rs2_intrinsics i, cinfo_t& cinfo)
{
    // Set height and width.
    cinfo.height = i.width;
    cinfo.width  = i.height;
    cinfo.distortion_model = "plumb_bob";

    // Set distortion and intrinsic parameters.
    cinfo.D.resize(5);
    std::fill(std::begin(cinfo.D), std::end(cinfo.D), 0.0);  // No distortion.

    // set K
    std::fill(std::begin(cinfo.K), std::end(cinfo.K), 0.0);
    cinfo.K[0] = i.fx;
    cinfo.K[2] = i.ppx;
    cinfo.K[4] = i.fy;
    cinfo.K[5] = i.ppy;
    cinfo.K[8] = 1.0;

    // Set cinfo.R to be an identity matrix.
    std::fill(std::begin(cinfo.R), std::end(cinfo.R), 0.0);
    cinfo.R[0] = cinfo.R[4] = cinfo.R[8] = 1.0;

    // Set 3x4 camera matrix.
    std::fill(std::begin(cinfo.P), std::end(cinfo.P), 0.0);
    cinfo.P[0]  = i.fx;
    cinfo.P[2]  = i.ppx;
    cinfo.P[5]  = i.fy;
    cinfo.P[6]  = i.ppy;
    cinfo.P[10] = 1.0;

    // No binning
    cinfo.binning_x = cinfo.binning_y = 0;

    // ROI is same as entire image.
    cinfo.roi.x_offset = cinfo.roi.y_offset = 0;
    cinfo.roi.width = cinfo.roi.height = 0;
    cinfo.roi.do_rectify = false;
}

void RealSenseCamera::publishCameraInfo()
{
    cinfo_t	color_cinfo;
    color_cinfo.header.stamp = ros::Time::now();
    color_cinfo.header.frame_id = "camera_color_optical_frame";
    toRosCameraInfo(getColorCameraIntrinsics(), color_cinfo);
    publishers_.color_camera_info.publish(color_cinfo);

    cinfo_t	depth_cinfo;
    depth_cinfo.header.stamp = ros::Time::now();
    depth_cinfo.header.frame_id = "camera_depth_optical_frame";
    toRosCameraInfo(getColorCameraIntrinsics(), depth_cinfo);
    publishers_.depth_camera_info.publish(depth_cinfo);
}

void RealSenseCamera::publishInfoCallback(ros::TimerEvent const &) 
{
    publishCameraInfo();
    publishStaticTransforms();
}

//###################################################### 
// Transform
//###################################################### 

void RealSenseCamera::publish_static_tf(const ros::Time& t, 
    const float3& trans, const quaternion& q, 
    const std::string& from, const std::string& to)
{
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = from;
    msg.child_frame_id = to;
    msg.transform.translation.x = trans.z;
    msg.transform.translation.y = -trans.x;
    msg.transform.translation.z = -trans.y;
    msg.transform.rotation.x = q.x;
    msg.transform.rotation.y = q.y;
    msg.transform.rotation.z = q.z;
    msg.transform.rotation.w = q.w;
    static_tf_broadcaster_.sendTransform(msg);
}

void RealSenseCamera::publishStaticTransforms()
{
    ros::Time transform_ts = ros::Time::now();
    float3 zero_trans{0, 0, 0};
    publish_static_tf(transform_ts, zero_trans, quaternion{0, 0, 0, 1}, "world", "camera_link");
}

}; // namespace o2as
