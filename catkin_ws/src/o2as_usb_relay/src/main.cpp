/*!
 *   \file	main.cpp
 */
#include <ros/ros.h>
#include "o2as_usb_relay/SetPower.h"
#include "TU/USB++.h"

namespace o2as_usb_relay
{
template <class BASE>
class USBRelay : public BASE
{
  private:
    using	super = BASE;

  public:
    USBRelay()
	:super(),
	 nh_("~"),
	 service_(nh_.advertiseService("set_power",
				       &USBRelay::setPower, this))
    {
	ROS_INFO("o2as_usb_relay_server is active.");
    }

    bool
    setPower(SetPower::Request& req, SetPower::Response& res)
    {
	ROS_INFO("Turning port %d %s.", req.port, (req.on ? "on" : "off"));

	try
	{
	    super::setPower(req.port, req.on);
	}
	catch (const std::exception& err)
	{
	    res.success = false;
	    res.message = err.what();

	    return false;
	}
		    
	res.success = true;
	res.message = std::string("Port ") + char('0' + req.port)
		    + " is turned " + (req.on ? "on." : "off.");

	return true;
    }

  private:
    ros::NodeHandle		nh_;
    const ros::ServiceServer	service_;
};

}	// namespace o2as_usb_relay

int
main(int argc, char** argv)
{
    using	USBRelay = o2as_usb_relay::USBRelay<TU::USBRelay>;
    
    ros::init(argc, argv, "o2as_usb_relay_server");

    try
    {
	o2as_usb_relay::USBRelay<TU::USBRelay>	relay;
	ros::spin();
    }
    catch (const std::exception& err)
    {
	ROS_INFO("%s", err.what());

	return 1;
    }

    return 0;
}
