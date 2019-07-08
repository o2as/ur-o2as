/*!
 *  \file	USBHid.cc
 */ 
#include "TU/USB++.h"

namespace TU
{
/************************************************************************
*  constants								*
************************************************************************/
static const u_int USB_RT_HID			= (USB_TYPE_CLASS |
						   USB_RECIP_DEVICE);
static const u_int USBRQ_HID_GET_REPORT		= 0x01;
static const u_int USBRQ_HID_SET_REPORT		= 0x09;
static const u_int USB_HID_REPORT_TYPE_FEATURE	= 3;
    
static const u_int CTRL_TIMEOUT			= 5000;

/************************************************************************
*  class USBHid								*
************************************************************************/
USBHid::USBHid(uint16_t idVendor, uint16_t idProduct, bool useReportIDs)
    :USBDevice(idVendor, idProduct), _useReportIDs(useReportIDs)
{
}

USBHid&
USBHid::setReport(const char *buffer, int len)
{
    const int	reportId = buffer[0];

    if (!_useReportIDs)
    {
        buffer++;   /* skip dummy report ID */
        len--;
    }
    if (usb_control_msg(handle(), USB_RT_HID | USB_ENDPOINT_OUT,
			USBRQ_HID_SET_REPORT,
			USB_HID_REPORT_TYPE_FEATURE << 8 | (reportId & 0xff),
			0, const_cast<char*>(buffer), len, CTRL_TIMEOUT) < 0)
	throw std::runtime_error(std::string("USBHid::setReport(): ") +
				 usb_strerror());
    return *this;
}

int
USBHid::getReport(int reportNumber, char* buffer, int maxLen)
{
    if (!_useReportIDs)
    {
        buffer++;	   // make room for dummy report ID
        maxLen--;
    }

    int	len = usb_control_msg(handle(), USB_RT_HID | USB_ENDPOINT_IN,
			      USBRQ_HID_GET_REPORT,
			      USB_HID_REPORT_TYPE_FEATURE << 8 | reportNumber,
			      0, buffer, maxLen, CTRL_TIMEOUT);

    if (len < 0)
	throw std::runtime_error(std::string("USBHid::getReport(): ") +
				 usb_strerror());

    if (!_useReportIDs)
    {
        buffer[-1] = reportNumber;	// add dummy report ID
        ++len;
    }

    return len;
}

}	// namespace TU
