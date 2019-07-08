/*
 *  $Id: USBHub.cc,v 1.1.1.1 2012-09-15 08:03:09 ueshiba Exp $
 */
#include "TU/USB++.h"

namespace TU
{
/************************************************************************
*  constants								*
************************************************************************/
static const u_int	USB_RT_HUB		= (USB_TYPE_CLASS |
						   USB_RECIP_DEVICE);
static const u_int	USB_RT_PORT		= (USB_TYPE_CLASS |
						   USB_RECIP_OTHER);
static const u_int	USB_PORT_FEAT_POWER	= 8;
static const u_int	USB_PORT_FEAT_INDICATOR	= 22;

static const u_int	HUB_CHAR_LPSM		= 0x0003;
static const u_int	HUB_CHAR_PORTIND	= 0x0080;

static const u_int	CTRL_TIMEOUT		= 1000;

/************************************************************************
*  class USBHub								*
************************************************************************/
USBHub::USBHub(uint16_t idVendor, uint16_t idProduct)
    :USBDevice(idVendor, idProduct), _nports(0)
{
    initialize();
}

USBHub&
USBHub::setPower(u_int port, bool on)
{
    setStatus((on ? USB_REQ_SET_FEATURE : USB_REQ_CLEAR_FEATURE),
	      USB_PORT_FEAT_POWER, port + 1);
    getStatus(port);

    return *this;
}

USBHub&
USBHub::setLED(u_int port, u_int value)
{
    return setStatus(USB_REQ_SET_FEATURE,
		     USB_PORT_FEAT_INDICATOR, (value << 8) | (port + 1));
}

bool
USBHub::isPowerOn(u_int port) const
{
    return getStatus(port) & 0x0100;
}

u_int
USBHub::getLED(u_int port) const
{
    return getStatus(port) & 0x1000;
}

/*
 *  private member functions
 */
void
USBHub::initialize()
{
    using namespace	std;
    
    struct Desc
    {
	u_char	len;
	u_char	type;
	u_char	nports;
	u_char	hub_chars[2];
	u_char	bPwrOn2PwrGood;
	u_char	bHubContrCurrent;
	u_char	data[0];
    };
    char	buf[1024];
    Desc*	desc = (Desc*)buf;
    int		len = usb_control_msg(handle(), USB_RT_HUB | USB_ENDPOINT_IN,
				      USB_REQ_GET_DESCRIPTOR,
				      USB_DT_HUB << 8, 0, 
				      buf, sizeof(buf), CTRL_TIMEOUT);
    if (len <= sizeof(Desc))
	throw runtime_error(string("USBHub::initialize(): failed to get usb descriptor!") + usb_strerror());
    
    _nports = desc->nports;
    
#if defined(_DEBUG)
    switch ((desc->hub_chars[0] & HUB_CHAR_LPSM))
    {
      case 0:
	cerr << " INFO: ganged switching." << endl;
	break;
      case 1:
	cerr << " INFO: individual power switching." << endl;
	break;
      default:
	cerr << " WARN: No power switching." << endl;
	break;
    }

    if (!(desc->hub_chars[0] & HUB_CHAR_PORTIND))
	cerr << " WARN: Port indicators are NOT supported." << endl;
#endif
}

USBHub&
USBHub::setStatus(u_int request, u_int feature, u_int index)
{
    using namespace	std;
    
    if (usb_control_msg(handle(), USB_RT_PORT | USB_ENDPOINT_OUT,
			request, feature, index, 0, 0, CTRL_TIMEOUT) < 0)
	throw runtime_error(string("USBHub::setStatus(): failed in usb_control_msg()!") + usb_strerror());
    
    return *this;
}

u_int32_t
USBHub::getStatus(u_int port) const
{
    using namespace	std;

    u_int32_t	status;
    
    if (usb_control_msg(handle(), USB_RT_PORT | USB_ENDPOINT_IN,
			USB_REQ_GET_STATUS, 0, port + 1,
			(char*)&status, sizeof(status), CTRL_TIMEOUT) < 0)
	throw runtime_error(string("USBHub::getStatus(): failed in usb_control_msg()!") + usb_strerror());
    
    return status;
}

/************************************************************************
*  global functions							*
************************************************************************/
std::ostream&
operator <<(std::ostream& out, const USBHub& hub)
{
    out << "idVendor: 0x"    << std::hex << hub.idVendor()
	<< ", idProduct: 0x" << hub.idProduct() << std::dec
	<< std::endl;
    
    for (u_int port = 0; port < hub.nports(); ++port)
    {
	u_int32_t	status = hub.getStatus(port);

	out << "Port " << port << ':'
	    << (status & 0x100000 ? " C_RESET"	 : "")
	    << (status & 0x080000 ? " C_OC"	 : "")
	    << (status & 0x040000 ? " C_SUSPEND" : "")
	    << (status & 0x020000 ? " C_ENABLE"	 : "")
	    << (status & 0x010000 ? " C_CONNECT" : "");

	out << (status &   0x1000 ? " indicator" : "")
	    << (status &   0x0800 ? " test"	 : "")
	    << (status &   0x0400 ? " highspeed" : "")
	    << (status &   0x0200 ? " lowspeed"	 : "")
	    << (status &   0x0100 ? " power"	 : "");
	
	out << (status &     0x10 ? " RESET"	 : "")
	    << (status &     0x08 ? " oc"	 : "")
	    << (status &     0x04 ? " suspend"	 : "")
	    << (status &     0x02 ? " enable"	 : "")
	    << (status &     0x01 ? " connect"	 : "")
	    << std::endl;
    }

    return out;
}

}
