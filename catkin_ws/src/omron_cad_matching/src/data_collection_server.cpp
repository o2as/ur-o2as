// ros
#include <ros/ros.h>

// image topic
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// opencv
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// cad
#include "omron_cad_matching/util_cam.h"
#include "omron_cad_matching/SaveFrame.h"

using namespace std;

namespace o2as {
namespace omron {

class DataCollectionServer
{
private:
    ros::NodeHandle nh_;
    ros::ServiceServer save_frame_;

public:
    DataCollectionServer() : nh_("~")
    {
        save_frame_  = nh_.advertiseService("save_frame", &DataCollectionServer::save_frame, this);
    }

    ~DataCollectionServer()
    {
    }

    void savePointCloud(std::string filename, sensor_msgs::PointCloud2& cloud_msg)
    {
        ROS_INFO_STREAM("save point cloud to :" << filename);
        int width = cloud_msg.width;
        int height = cloud_msg.height;
        float *point_cloud = (float*)malloc(width * height * 3 * sizeof(float));
        GetPointCloudBinary(cloud_msg, point_cloud);
        SavePointCloudToBinary(point_cloud, width, height, (char*)filename.c_str());
        if (point_cloud != NULL) free(point_cloud);
    }

    void saveImage(std::string filename, sensor_msgs::Image& image_msg)
    {
        ROS_INFO_STREAM("save image to :" << filename);
        cv::Mat cv_image = cv_bridge::toCvCopy(image_msg, image_msg.encoding)->image;
        cv::imwrite((char*)filename.c_str(), cv_image);
    }

    bool save_frame(
        omron_cad_matching::SaveFrame::Request &req,
        omron_cad_matching::SaveFrame::Response &res)
    {
        savePointCloud(req.cloud_filename, req.cloud);
        saveImage(req.image_filename, req.image);
        return true;
    }
};

} // namespace omron
} // namespace o2as

using namespace o2as::omron;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_collection_server");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
    DataCollectionServer node;
    ros::spin();
    return 0;
}
