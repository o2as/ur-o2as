/*
 *  $Id$
 */
#include <stdexcept>
#include <sstream>
#include "TU/USB++.h"

namespace TU
{
/************************************************************************
*  static functions							*
************************************************************************/
static usb_dev_handle*
usb_get_handle(uint16_t idVendor, uint16_t idProduct)
{
    for (usb_bus* bus = usb_get_busses(); bus; bus = bus->next)
	for (struct usb_device* dev = bus->devices; dev; dev = dev->next)
	    if (dev->descriptor.idVendor  == idVendor &&
		dev->descriptor.idProduct == idProduct)
		return usb_open(dev);

    return nullptr;
}

/************************************************************************
*  class USBDevice							*
************************************************************************/
USBDevice::Initializer	USBDevice::_initializer;
    
USBDevice::USBDevice(uint16_t idVendor, uint16_t idProduct)
    :_handle(usb_get_handle(idVendor, idProduct))
{
    if (!_handle)
    {
	using namespace	std;
    
	ostringstream	s;
	s << "USBDevice::USBDevice(): failed to open device(idVendor: 0x"
	  << hex << idVendor << ", idProduct: 0x" << idProduct << ")!";
	throw runtime_error(s.str());
    }
}

void
USBDevice::listup(std::ostream& out, uint8_t deviceClass)
{
    for (usb_bus* bus = usb_get_busses(); bus; bus = bus->next)
	for (struct usb_device* dev = bus->devices; dev; dev = dev->next)
	    if (dev->descriptor.bDeviceClass == deviceClass)
		out << "Device at "
		    << bus->dirname << ':' << int(dev->devnum)
		    << std::endl;
}

}	// nbamespace TU

