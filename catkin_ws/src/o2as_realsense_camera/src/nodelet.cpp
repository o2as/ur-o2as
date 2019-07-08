#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <o2as_realsense_camera/RealSenseCamera.h>

namespace o2as{

class RealSenseCameraNodelet: public nodelet::Nodelet
{
public:
    RealSenseCameraNodelet() {}

private:
    virtual void onInit(void);
	boost::shared_ptr<RealSenseCamera> pimpl;
};

void RealSenseCameraNodelet::onInit(void)
{
	// // set logger level to debug
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    // ros::console::notifyLoggerLevelsChanged();

	ros::NodeHandle nh = getNodeHandle();
	ROS_INFO_STREAM("realsense camera nodelet is loaded.");
	pimpl.reset(new RealSenseCamera(nh, getPrivateNodeHandle()));
}

} // namespace o2as

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(o2as::RealSenseCameraNodelet, nodelet::Nodelet);
