/*!
 *  \file	main.cpp
 */
#include <string>
#include <array>
#include <limits>
#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>

#include <PhoXi.h>

#include <o2as_phoxi_camera/o2as_phoxi_cameraConfig.h>
#include <o2as_phoxi_camera/PhoXiSize.h>
#include <o2as_phoxi_camera/SetString.h>
#include <o2as_phoxi_camera/GetString.h>
#include <o2as_phoxi_camera/GetStringList.h>
#include <o2as_phoxi_camera/GetInt.h>
#include <o2as_phoxi_camera/GetFrame.h>
#include <o2as_phoxi_camera/DumpFrame.h>
#include <o2as_phoxi_camera/GetSupportedCapturingModes.h>

#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

namespace o2as_phoxi_camera
{

static std::ostream&
operator <<(std::ostream& out, const o2as_phoxi_cameraConfig& config)
{
    out << "resolution:\t\t"		<< config.resolution
	<< "\nscan multiplier:\t"	<< config.scan_multiplier
	<< "\nshutter multiplier:\t"	<< config.shutter_multiplier
	<< "\ntrigger mode:\t\t"	<< config.trigger_mode
	<< "\ntimeout:\t\t"		<< config.timeout
	<< "\nconfidence:\t\t"		<< config.confidence
	<< "\nsend point cloud:\t"	<< config.send_point_cloud
	<< "\nsend normal map:\t"	<< config.send_normal_map
	<< "\nsend depth map:\t\t"	<< config.send_depth_map
	<< "\nsend confidence map:\t"	<< config.send_confidence_map
	<< "\nsend texture:\t\t"	<< config.send_texture
	<< "\npoint_format:\t\t"	<< config.point_format
	<< "\nintensity_scale:\t"	<< config.intensity_scale;
}

/************************************************************************
*   class Camera                                                        *
************************************************************************/

class Camera
{
  private:
    using cloud_t = sensor_msgs::PointCloud2;
    using image_t = sensor_msgs::Image;
    using cinfo_t = sensor_msgs::CameraInfo;

    enum
    {
	XYZ = 0, XYZRGB = 1, XYZI = 2
    };

  public:
		Camera()						;
		~Camera()						;

    void	run()							;

  private:
    void	reconf_callback(o2as_phoxi_cameraConfig& config,
				uint32_t level)				;
    bool	get_device_list(GetStringList::Request&  req, 
				GetStringList::Response& res)		;
    bool	is_acquiring(std_srvs::Trigger::Request&  req, 
			     std_srvs::Trigger::Response& res)		;
    bool	start_acquisition(std_srvs::Trigger::Request&  req, 
				  std_srvs::Trigger::Response& res)	;
    bool	stop_acquisition(std_srvs::Trigger::Request&  req, 
				 std_srvs::Trigger::Response& res)	;
    bool	trigger_frame(GetInt::Request&  req,
			      GetInt::Response& res)			;
    bool	get_frame(GetFrameRequest& req, GetFrameResponse& res)	;
    bool	save_frame(SetString::Request&  req,
			   SetString::Response& res)			;
    bool	flush_buffer(std_srvs::Trigger::Request&  req,
			     std_srvs::Trigger::Response& res)		;
    bool	get_supported_capturing_modes(
			GetSupportedCapturingModes::Request&  req, 
			GetSupportedCapturingModes::Response& res)	;
    bool	get_hardware_identification(GetString::Request&  req, 
					    GetString::Response& res)	;
    template <class T>
    static bool	save_image(const std::string& filename,
			   const pho::api::Mat2D<T>& phoxi_image,
			   float scale)					;
    template <class T>
    void	get_cloud(const pho::api::PointCloud32f& phoxi_cloud,
			  const pho::api::Mat2D<T>& phoxi_texture,
			  const ros::Publisher& publisher,
			  const ros::Time& stamp,
			  const float distanceScale,
			  const float intensityScale,
			  cloud_t& cloud, bool publish)		const	;
    template <class T>
    static void	get_image(const pho::api::Mat2D<T>& phoxi_image,
			  const ros::Publisher& publisher,
			  const ros::Time& stamp,
			  const std::string& encoding, 
			  typename T::ElementChannelType scale,
			  image_t& image,
			  bool publish)					;
    void	get_frame_msgs(GetFrameResponse& res,
			       bool publish)			const	;
    void	publish_camera_info()				const	;
    void	flush_buffer_internal()					;

  private:
    ros::NodeHandle			_nh;

    mutable std::mutex			_mutex;
    const pho::api::PhoXiFactory	_factory;
    pho::api::PPhoXi			_device;
    pho::api::PFrame			_frame;
    int					_frameId;	// unique id of _frame
    std::string				_frameName;	// frame id used by tf
    std::array<double, 9>		_K;
    int					_pointFormat;
    float				_intensityScale;
    
    const dynamic_reconfigure::Server<o2as_phoxi_cameraConfig> _reconf_server;

    const ros::ServiceServer		_get_device_list_server;
    const ros::ServiceServer		_is_acquiring_server;
    const ros::ServiceServer		_start_acquisition_server;
    const ros::ServiceServer		_stop_acquisition_server;
    const ros::ServiceServer		_trigger_frame_server;
    const ros::ServiceServer		_get_frame_server;
    const ros::ServiceServer		_save_frame_server;
    const ros::ServiceServer		_flush_buffer_server;
    const ros::ServiceServer		_get_hardware_identification_server;
    const ros::ServiceServer		_get_supported_capturing_modes_server;

    const ros::Publisher		_cloud_publisher;
    const ros::Publisher		_normal_map_publisher;
    const ros::Publisher		_depth_map_publisher;
    const ros::Publisher		_confidence_map_publisher;
    const ros::Publisher		_texture_publisher;
    const ros::Publisher		_camera_info_publisher;
};
    
Camera::Camera()
    :_nh("~"),
     _factory(),
     _device(nullptr),
     _frame(nullptr),
     _frameId(-1),
     _frameName("map"),
     _K({2215.13350577,    0.0        , 1030.47471121 ,
	    0.0       , 2215.13350577 ,  756.735726174,
            0.0       ,    0.0        ,    1.0        }),
     _pointFormat(0),
     _intensityScale(255.0/4095.0),
     _reconf_server(_nh),
     _get_device_list_server(
	 _nh.advertiseService("get_device_list",   &get_device_list,   this)),
     _is_acquiring_server(
	 _nh.advertiseService("is_acquiring",	   &is_acquiring,      this)),
     _start_acquisition_server(
	 _nh.advertiseService("start_acquisition", &start_acquisition, this)),
     _stop_acquisition_server(
	 _nh.advertiseService("stop_acquisition",  &stop_acquisition,  this)),
     _trigger_frame_server(
	 _nh.advertiseService("trigger_frame",	   &trigger_frame,     this)),
     _get_frame_server(
	 _nh.advertiseService("get_frame",	   &get_frame,	       this)),
     _save_frame_server(
	 _nh.advertiseService("save_frame",	   &save_frame,	       this)),
     _flush_buffer_server(
	 _nh.advertiseService("flush_buffer",	   &flush_buffer,      this)),
     _get_hardware_identification_server(
	 _nh.advertiseService("get_hardware_identification",
			      &get_hardware_identification, this)),
     _get_supported_capturing_modes_server(
	 _nh.advertiseService("get_supported_capturing_modes",
			      &get_supported_capturing_modes, this)),
     _cloud_publisher(	       _nh.advertise<cloud_t>("pointcloud",	1)),
     _normal_map_publisher(    _nh.advertise<image_t>("normal_map",	1)),
     _depth_map_publisher(     _nh.advertise<image_t>("depth_map",	1)),
     _confidence_map_publisher(_nh.advertise<image_t>("confidence_map", 1)),
     _texture_publisher(       _nh.advertise<image_t>("texture",	1)),
     _camera_info_publisher(   _nh.advertise<cinfo_t>("camera_info",	1))
{
  // Search for a device with specified ID.
    std::string	id;
    _nh.param<std::string>("id", id,
			   "InstalledExamples-PhoXi-example(File3DCamera)");
    for (size_t pos; (pos = id.find('\"')) != std::string::npos; ) 
	id.erase(pos, 1);

    if (!_factory.isPhoXiControlRunning())
    {
	ROS_ERROR_STREAM("PhoXiControll is not running.");
	throw std::runtime_error("");
    }
    
    for (const auto& devinfo : _factory.GetDeviceList())
	if (devinfo.HWIdentification == id)
	{
	    _device = _factory.Create(devinfo);
	    break;
	}
    if (!_device)
    {
	ROS_ERROR_STREAM("Failed to find camera[" << id << "].");
	throw std::runtime_error("");
    }

  // Connect to the device.
    if (!_device->Connect())
    {
	ROS_ERROR_STREAM("Failed to open camera[" << id << "].");
	throw std::runtime_error("");
    }

  // Stop acquisition.
    _device->StopAcquisition();

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") Initializing configuration.");

  // Set parameter values according to the current device state.
    auto modes = _device->SupportedCapturingModes.GetValue();
    int idx = 0;
    for (int i = 0; i < modes.size(); ++i)
	if (modes[i] == _device->CapturingMode)
	{
	    idx = i;
	    break;
	}
    ros::param::set("resolution", idx + 1);
    ros::param::set("scan_multiplier",
		    _device->CapturingSettings->ScanMultiplier);
    ros::param::set("shutter_multiplier",
		    _device->CapturingSettings->ShutterMultiplier);
    ros::param::set("trigger_mode",
		    pho::api::PhoXiTriggerMode::Value(
			_device->TriggerMode.GetValue()));
    ros::param::set("timeout",
		    int(_device->Timeout.GetValue()));
    ros::param::set("confidence",
		    _device->ProcessingSettings->Confidence);
    ros::param::set("send_point_cloud",
		    _device->OutputSettings->SendPointCloud);
    ros::param::set("send_normal_map",
		    _device->OutputSettings->SendNormalMap);
    ros::param::set("send_depth_map",
		    _device->OutputSettings->SendDepthMap);
    ros::param::set("send_confidence_map",
		    _device->OutputSettings->SendConfidenceMap);
    ros::param::set("send_texture",
		    _device->OutputSettings->SendTexture);
    ros::param::set("intensity_scale",
		    _intensityScale);
    _reconf_server.setCallback(boost::bind(&reconf_callback, this, _1, _2));

  // Read camera intrinsic parameters.
    const std::string
	intrinsic_filename = ros::package::getPath("o2as_phoxi_camera")
			   + "/config/"
			   + _device->HardwareIdentification.GetValue()
			   + ".yaml";
    try
    {
	YAML::Node	intrinsic = YAML::LoadFile(intrinsic_filename);
	const auto	K = intrinsic["K"].as<std::vector<double> >();
	std::copy_n(K.cbegin(), _K.size(), _K.begin());
    }
    catch (const std::exception& err)
    {
	ROS_WARN_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") failed to open " << intrinsic_filename);
    }

  // Set frame name.
    _nh.param<std::string>("frame", _frameName, "map");
    
  // Start acquisition.
    flush_buffer_internal();
    _device->StartAcquisition();

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") o2as_phoxi_camera is active.");
}

Camera::~Camera()
{
    if (_device && _device->isConnected())
    {
	_device->StopAcquisition();
	_device->Disconnect();
    }
}

void
Camera::run()
{
    ros::Rate looprate(10);
        
    while (ros::ok())
    {
	using namespace	pho::api;
	
	if (_device->TriggerMode == PhoXiTriggerMode::Freerun &&
	    _device->isAcquiring())
	{
	    std::lock_guard<std::mutex>	lock(_mutex);
                
	    if ((_frame = _device->GetFrame(PhoXiTimeout::ZeroTimeout)) &&
		_frame->Successful)
	    {
		GetFrameResponse	res;
		get_frame_msgs(res, true);
	    }
	}

	publish_camera_info();
                
	ros::spinOnce();
	looprate.sleep();
    }
}

void
Camera::reconf_callback(o2as_phoxi_cameraConfig& config, uint32_t level)
{
  //std::cerr << config << std::endl;
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") callback called. level = "
		    << level);

    if (level & (1 << 4))
    {
	const auto acq = _device->isAcquiring();
	if (acq)
	    _device->StopAcquisition();

	const auto modes = _device->SupportedCapturingModes.GetValue();
	if (config.resolution < modes.size())
	    _device->CapturingMode = modes[config.resolution];
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set resolution to "
			<< _device->CapturingMode.GetValue().Resolution.Width
			<< 'x' 
			<< _device->CapturingMode.GetValue().Resolution.Height);

	flush_buffer_internal();
	if (acq)
	    _device->StartAcquisition();
    }
        
    if (level & (1 << 5))
    {
	_device->CapturingSettings->ScanMultiplier
	    = config.scan_multiplier;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set scan multiplier to "
			<< _device->CapturingSettings->ScanMultiplier);
    }

    if (level & (1 << 6))
    {
	_device->CapturingSettings->ShutterMultiplier
	    = config.shutter_multiplier;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set shutter multiplier to "
			<< _device->CapturingSettings->ShutterMultiplier);
    }

    if (level & (1 << 7))
    {
	const auto acq = _device->isAcquiring();
	if (acq)
	    _device->StopAcquisition();

	_device->TriggerMode = config.trigger_mode;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set trigger mode to "
			<< _device->TriggerMode.GetValue());
	flush_buffer_internal();

	if (acq)
	    _device->StartAcquisition();
    }

    if (level & (1 << 8))
    {
	_device->Timeout = config.timeout;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set timeout to "
			<< _device->Timeout.GetValue());
    }

    if (level & (1 << 9))
    {
	_device->ProcessingSettings->Confidence = config.confidence;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set confidence to "
			<< _device->ProcessingSettings->Confidence);
    }

    if (level & (1 << 10))
    {
	_device->OutputSettings->SendPointCloud = config.send_point_cloud;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") "
			<< (_device->OutputSettings->SendPointCloud ?
			    "enabled" : "disabled")
			<< " publishing point cloud");
    }

    if (level & (1 << 11))
    {
	_device->OutputSettings->SendNormalMap = config.send_normal_map;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") "
			<< (_device->OutputSettings->SendNormalMap ?
			    "enabled" : "disabled")
			<< " publishing normal map");
    }

    if (level & (1 << 12))
    {
	_device->OutputSettings->SendDepthMap = config.send_depth_map;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") "
			<< (_device->OutputSettings->SendDepthMap ?
			    "enabled" : "disabled")
			<< " publishing depth map");
    }

    if (level & (1 << 13))
    {
	_device->OutputSettings->SendConfidenceMap
	    = config.send_confidence_map;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") "
			<< (_device->OutputSettings->SendConfidenceMap ?
			    "enabled" : "disabled")
			<< " publishing confidence map");
    }

    if (level & (1 << 14))
    {
	_device->OutputSettings->SendTexture = config.send_texture;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") "
			<< (_device->OutputSettings->SendTexture ?
			    "enabled" : "disabled")
			<< " publishing texture map");
    }

    if (level & (1 << 15))
    {
	_pointFormat = config.point_format;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set point format to "
			<< _pointFormat);
    }

    if (level & (1 << 16))
    {
	_intensityScale = config.intensity_scale;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") set intensity scale to "
			<< _intensityScale);
    }
}

bool
Camera::get_device_list(GetStringList::Request&  req, 
			GetStringList::Response& res)
{
    _factory.StartConsoleOutput("Admin-On");

    const auto	devinfos = _factory.GetDeviceList();

    res.len = devinfos.size();
    for (const auto& devinfo : devinfos)
	res.out.push_back(devinfo.HWIdentification);

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") get_device_list: succeded.");

    return true;
}

bool
Camera::is_acquiring(std_srvs::Trigger::Request&  req, 
		     std_srvs::Trigger::Response& res)
{
    res.success = _device->isAcquiring();
    res.message = (res.success ? "yes" : "no");

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") is_acquiring: "
		    << res.message);

    return true;
}

bool
Camera::start_acquisition(std_srvs::Trigger::Request&  req, 
			  std_srvs::Trigger::Response& res)
{
    if (_device->isAcquiring())
    {
	res.success = true;
	res.message = "already in aquisition.";
    }
    else
    {
      // Flush buffer to avoid publishing data during past acquisition.
	flush_buffer_internal();	// MUST!!!
	res.success = _device->StartAcquisition();
	res.message = (res.success ? "scceeded." : "failed.");
    }
        
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") start_acquisition: "
		    << res.message);

    ros::Duration(1.0).sleep();
    return true;
}

bool
Camera::stop_acquisition(std_srvs::Trigger::Request&  req, 
			 std_srvs::Trigger::Response& res)
{
    if (_device->isAcquiring())
    {
	res.success = _device->StopAcquisition();
	res.message = (res.success ? "succeeded." : "failed.");
    }
    else
    {
	res.success = true;
	res.message = "already not in aquisition.";
    }
        
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") stop_acquisition: "
		    << res.message);

    return true;
}

bool
Camera::trigger_frame(GetInt::Request& req, GetInt::Response& res)
{
  //flush_buffer_internal();
    
    for (int n = 0; n < 10; ++n)
    {
      // Wait until the device be ready for accepting trigger command.
      // (1st arg.)
      // The call is blocked until the device finishing grab. (2nd arg.)
	res.out = _frameId = _device->TriggerFrame(true, true);
	
	switch (_frameId)
	{
	  case -1:
	    ROS_WARN_STREAM('('
			    << _device->HardwareIdentification.GetValue()
			    << ") trigger_frame: not accepted.");
	    break;
	  case -2:
	    ROS_ERROR_STREAM('('
			     << _device->HardwareIdentification.GetValue()
			     << ") trigger_frame: device is not running.");
	    return true;
	  case -3:
	    ROS_ERROR_STREAM('('
			     << _device->HardwareIdentification.GetValue()
			     << ") trigger_frame: communication error.");
	    return true;
	  case -4:
	    ROS_ERROR_STREAM('('
			     << _device->HardwareIdentification.GetValue()
			     << ") trigger_frame: WaitForGrabbingEnd is not supported.");
	    return true;
	  default:
	    ROS_INFO_STREAM('('
			    << _device->HardwareIdentification.GetValue()
			    << ") trigger_frame: succeeded. [frame #"
			    << _frameId
			    << ']');
	    return true;
	}
    }

    return true;
}

bool
Camera::get_frame(GetFrameRequest& req, GetFrameResponse& res)
{
    using namespace	pho::api;
    
    std::lock_guard<std::mutex>	lock(_mutex);

    if (_frameId < 0)
    {
	res.success = false;
	ROS_WARN_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") get_frame: failed. [not triggered yet]");
	return true;
    }

    while ((_frame = _device->GetFrame(PhoXiTimeout::ZeroTimeout)) != nullptr)
	if (_frame->Successful && _frame->Info.FrameIndex == _frameId)
	{
	    get_frame_msgs(res, req.publish);
	    res.success = true;
	    ROS_INFO_STREAM('('
			    << _device->HardwareIdentification.GetValue()
			    << ") get_frame: succeeded. [frame #"
			    << _frameId
			    << ']');
	    _frameId = -1;

	    return true;
	}
    
    res.success = false;
    ROS_ERROR_STREAM('('
		     << _device->HardwareIdentification.GetValue()
		     << ") get_frame: failed. [not found frame #"
		     << _frameId
		     << ']');
    _frameId = -1;

    return true;
}

bool
Camera::save_frame(SetString::Request& req, SetString::Response& res)
{
    std::lock_guard<std::mutex>	lock(_mutex);

    if (_frame == nullptr || !_frame->Successful)
    {
	res.success = false;
	ROS_ERROR_STREAM('('
			 << _device->HardwareIdentification.GetValue()
			 << ") save_frame: failed. [no frame data]");
    }
    else if (!_frame->SaveAsPly(req.in + ".ply"))
    {
	res.success = false;
	ROS_ERROR_STREAM('('
			 << _device->HardwareIdentification.GetValue()
			 << ") save_frame: failed to save PLY to "
			 << req.in + ".ply");
    }
    else
    {
	res.success = true;
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") save_frame: succeeded to save PLY to "
			<< req.in + ".ply");
    }
    
    return true;
}

bool
Camera::flush_buffer(std_srvs::Trigger::Request&  req,
		     std_srvs::Trigger::Response& res)
{
    flush_buffer_internal();
    res.success = true;
    res.message = "succeeded.";
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") flush_buffer: "
		    << res.message);

    return true;
}
    
bool
Camera::get_supported_capturing_modes(
		GetSupportedCapturingModes::Request&  req, 
	        GetSupportedCapturingModes::Response& res)
{
    const auto	modes = _device->SupportedCapturingModes.GetValue();
    for (const auto& mode : modes)
    {
	o2as_phoxi_camera::PhoXiSize	size;
	size.Width  = mode.Resolution.Width;
	size.Height = mode.Resolution.Height;
	res.supported_capturing_modes.push_back(size);
    }

    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    << ") get_supported_capturing_moddes: succeeded.");

    return true;
}

bool
Camera::get_hardware_identification(GetString::Request&  req, 
				    GetString::Response& res)
{
    res.out = _device->HardwareIdentification;

    return true;
}

template <class T> void
Camera::get_cloud(const pho::api::PointCloud32f& phoxi_cloud,
		  const pho::api::Mat2D<T>& phoxi_texture,
		  const ros::Publisher& publisher,
		  const ros::Time& stamp,
		  const float distanceScale,
		  const float intensityScale,
		  cloud_t& cloud, bool publish) const
{
    using namespace	sensor_msgs;
    
    if (phoxi_cloud.Empty())
	return;

  // Convert pho::api::PointCloud32f to sensor_msgs::PointCloud2
    cloud.is_bigendian  = false;
    cloud.is_dense	= false;

    PointCloud2Modifier	modifier(cloud);
#if 0
    if (_pointFormat == XYZ)
	modifier.setPointCloud2Fields(3,
				      "x", 1, PointField::FLOAT32,
				      "y", 1, PointField::FLOAT32,
				      "z", 1, PointField::FLOAT32);
    else
	modifier.setPointCloud2Fields(4,
				      "x",   1, PointField::FLOAT32,
				      "y",   1, PointField::FLOAT32,
				      "z",   1, PointField::FLOAT32,
				      "rgb", 1, PointField::FLOAT32);
#else
    if (_pointFormat == XYZ)
	modifier.setPointCloud2FieldsByString(1, "xyz");
    else
	modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
#endif
    modifier.resize(phoxi_cloud.Size.Height * phoxi_cloud.Size.Width);

    cloud.header.stamp	  = stamp;
    cloud.header.frame_id = _frameName;
    cloud.height	  = phoxi_cloud.Size.Height;
    cloud.width		  = phoxi_cloud.Size.Width;
    cloud.row_step	  = cloud.width * cloud.point_step;
    
    for (int v = 0; v < cloud.height; ++v)
    {
	PointCloud2Iterator<float>	xyz(cloud, "x");
	xyz += cloud.width * v;
	
	for (int u = 0; u < cloud.width; ++u) 
	{
	    const auto&	p = phoxi_cloud.At(v, u);
		
	    if (float(p.z) == 0.0f)
	    {
		xyz[0] = xyz[1] = xyz[2]
		       = std::numeric_limits<float>::quiet_NaN();
	    }
	    else
	    {
		xyz[0] = p.x * distanceScale;
		xyz[1] = p.y * distanceScale;
		xyz[2] = p.z * distanceScale;
	    }

	    ++xyz;
	}

	if (_pointFormat == XYZRGB)
	{
	    PointCloud2Iterator<uint8_t>	rgb(cloud, "rgb");
	    rgb += cloud.width * v;
	
	    for (int u = 0; u < cloud.width; ++u) 
	    {
		const auto	val = phoxi_texture.At(v, u) * intensityScale;
		
		rgb[0] = rgb[1] = rgb[2] = (val > 255 ? 255 : val);
		++rgb;
	    }
	}
	else if (_pointFormat == XYZI)
	{
	    PointCloud2Iterator<float>	rgb(cloud, "rgb");
	    rgb += cloud.width * v;
	
	    for (int u = 0; u < cloud.width; ++u) 
	    {
		*rgb = phoxi_texture.At(v, u) * intensityScale;
		++rgb;
	    }
	}
    }
    
    if (publish)
	publisher.publish(cloud);
}

template <class T> void
Camera::get_image(const pho::api::Mat2D<T>& phoxi_image,
		  const ros::Publisher& publisher,
		  const ros::Time& stamp,
		  const std::string& encoding, 
		  typename T::ElementChannelType scale,
		  image_t& image,
		  bool publish)
{
    using namespace	sensor_msgs;
    using		element_ptr = const typename T::ElementChannelType*;
        
    if (phoxi_image.Empty())
	return;

    image.header.stamp    = stamp;
    image.header.frame_id = "camera";
    image.encoding	  = encoding;
    image.is_bigendian    = 0;
    image.height	  = phoxi_image.Size.Height;
    image.width		  = phoxi_image.Size.Width;
    image.step		  = image.width
			  *  image_encodings::numChannels(image.encoding)
			  * (image_encodings::bitDepth(image.encoding)/8);
    image.data.resize(image.step * image.height);

    const auto	p = reinterpret_cast<element_ptr>(phoxi_image[0]);
    const auto	q = reinterpret_cast<element_ptr>(
			phoxi_image[phoxi_image.Size.Height]);
	
    if (image.encoding == image_encodings::MONO8)
	std::transform(p, q, 
		       reinterpret_cast<uint8_t*>(image.data.data()),
		       [scale](const auto& x)->uint8_t
		       { auto y = scale * x; return y > 255 ? 255 : y; });
    else if (image.encoding == image_encodings::MONO16)
	std::transform(p, q, 
		       reinterpret_cast<uint16_t*>(image.data.data()),
		       [scale](const auto& x)->uint16_t
		       { return scale * x; });
    else if (image.encoding.substr(0, 4) == "32FC")
	std::transform(p, q,
		       reinterpret_cast<float*>(image.data.data()),
		       [scale](const auto& x)->float
		       { return scale * x; });
    else if (image.encoding.substr(0, 4) == "64FC")
	std::transform(p, q,
		       reinterpret_cast<double*>(image.data.data()),
		       [scale](const auto& x)->double
		       { return scale * x; });
    else
    {
	ROS_ERROR_STREAM("Unsupported image type!");
	throw std::logic_error("");
    }

    if (publish)
	publisher.publish(image);
}

void
Camera::get_frame_msgs(GetFrameResponse& res, bool publish) const
{
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue() << ") "
		    << "PointCloud: "
		    << _frame->PointCloud.Size.Width << 'x'
		    << _frame->PointCloud.Size.Height
		    << " [frame #" << _frame->Info.FrameIndex << ']');

  // Common setting.
    constexpr float	distanceScale = 0.001;
    const auto	timeNow = ros::Time::now();

  // Get point cloud.
    get_cloud(_frame->PointCloud, _frame->Texture, _cloud_publisher,
	      timeNow, distanceScale, _intensityScale, res.cloud, publish);

  // Get normal_map, depth_map, confidence_map and texture.
    get_image(_frame->NormalMap, _normal_map_publisher, timeNow,
	      sensor_msgs::image_encodings::TYPE_32FC3,
	      1, res.normal_map, publish);
    get_image(_frame->DepthMap, _depth_map_publisher, timeNow,
	      sensor_msgs::image_encodings::TYPE_32FC1,
	      distanceScale, res.depth_map, publish);
    get_image(_frame->ConfidenceMap, _confidence_map_publisher, timeNow,
	      sensor_msgs::image_encodings::TYPE_32FC1,
	      1, res.confidence_map, publish);
    get_image(_frame->Texture, _texture_publisher, timeNow,
	      sensor_msgs::image_encodings::MONO8,
	      _intensityScale, res.texture, publish);
}
    
void
Camera::publish_camera_info() const
{
    cinfo_t	cinfo;

  // Set header.
    cinfo.header.stamp    = ros::Time::now();
    cinfo.header.frame_id = "camera";

  // Set height and width.
    const auto	mode = _device->CapturingMode.GetValue();
    cinfo.height = mode.Resolution.Height;
    cinfo.width  = mode.Resolution.Width;

  // Set distortion and intrinsic parameters.
    cinfo.distortion_model = "plumb_bob";
#if 0
    const auto& calib = _device->CalibrationSettings.GetValue();
    const auto& d     = calib.DistortionCoefficients;
    const auto& K     = calib.CameraMatrix;
    cinfo.D.resize(d.size());
    std::copy(std::begin(d), std::end(d), std::begin(cinfo.D));
    std::copy(K[0], K[3], std::begin(cinfo.K));
#else    
    std::fill(std::begin(cinfo.D), std::end(cinfo.D), 0.0);  // No distortion.
    std::copy(std::begin(_K), std::end(_K), std::begin(cinfo.K));
#endif

  // Set cinfo.R to be an identity matrix.
    std::fill(std::begin(cinfo.R), std::end(cinfo.R), 0.0);
    cinfo.R[0] = cinfo.R[4] = cinfo.R[8] = 1.0;

  // Set 3x4 camera matrix.
    for (int i = 0; i < 3; ++i) 
	for (int j = 0; j < 3; ++j) 
	    cinfo.P[4*i + j] = cinfo.K[3*i + j];
    cinfo.P[3] = cinfo.P[7] = cinfo.P[11] = 0.0;
        
  // No binning
    cinfo.binning_x = cinfo.binning_y = 0;

  // ROI is same as entire image.
    cinfo.roi.width = cinfo.roi.height = 0;

    _camera_info_publisher.publish(cinfo);
}

void
Camera::flush_buffer_internal()
{
    using namespace	pho::api;
    
    for (PFrame frame;
	 (frame = _device->GetFrame(PhoXiTimeout::ZeroTimeout)) != nullptr; )
	ROS_INFO_STREAM('('
			<< _device->HardwareIdentification.GetValue()
			<< ") flushing buffer... [frame #"
			<< frame->Info.FrameIndex << ']');
    ROS_INFO_STREAM('('
		    << _device->HardwareIdentification.GetValue()
		    <<") finished flushing buffer!");

    _frameId = -1;
}
    
}	// namespace o2as_phoxi_camera

/************************************************************************
*   global functions                                                    *
************************************************************************/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "o2as_phoxi_camera");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				   ros::console::levels::Debug);

    try
    {
        o2as_phoxi_camera::Camera camera;
        camera.run();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
        return 1;
    }

    return 0;
}
