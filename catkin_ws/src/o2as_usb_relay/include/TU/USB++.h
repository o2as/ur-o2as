/*
 *  $Id: USB++.h,v 1.1.1.1 2012-09-15 08:03:09 ueshiba Exp $
 */
#ifndef TU_USBPP_H
#define TU_USBPP_H
#include <sys/types.h>
#include <iostream>
#include <usb.h>

namespace TU
{
/************************************************************************
*  class USBDevice							*
************************************************************************/
class USBDevice
{
  private:
    struct Initializer
    {
	Initializer()	{ usb_init(); usb_find_busses(); usb_find_devices(); }
    };

  protected:
    USBDevice(uint16_t idVendor, uint16_t idProduct)			;
    ~USBDevice()							;

  public:
    uint16_t		idVendor()				const	;
    uint16_t		idProduct()				const	;
    static void		listup(std::ostream& out, uint8_t deviceClass)	;

  protected:
    usb_dev_handle*	handle()		const	{ return _handle; }
	 
  private:
    usb_dev_handle* const	_handle;	//!< USBデバイスのハンドル

    static Initializer		_initializer;
};
    
inline
USBDevice::~USBDevice()
{
    if (_handle)
	usb_close(_handle);
}

inline uint16_t
USBDevice::idVendor() const
{
    return usb_device(_handle)->descriptor.idVendor;
}
    
inline uint16_t
USBDevice::idProduct() const
{
    return usb_device(_handle)->descriptor.idProduct;
}
    
/************************************************************************
*  class USBHub								*
************************************************************************/
class USBHub : public USBDevice
{
  public:
    USBHub(uint16_t idVendor=0x2101, uint16_t idProduct=0x8500)		;

    u_int	nports()					const	;
    USBHub&	setPower(u_int port, bool on)				;
    USBHub&	setLED(u_int port, u_int value)				;
    bool	isPowerOn(u_int port)				const	;
    u_int	getLED(u_int port)				const	;
    
    friend std::ostream&
		operator <<(std::ostream& out, const USBHub& hub)	;
    
  private:
    void	initialize()						;
    USBHub&	setStatus(u_int request, u_int feature, u_int index)	;
    uint32_t	getStatus(u_int port)				const	;
    
  private:
    u_int	_nports;	//!< USBハブのポート数
};

inline u_int
USBHub::nports() const
{
    return _nports;
}

/************************************************************************
*  class USBHid								*
************************************************************************/
class USBHid : public USBDevice
{
  public:
    USBHid(uint16_t idVendor, uint16_t idProduct, bool useReportIDs)	;

    USBHid&	setReport(const char *buffer, int len)			;
    int		getReport(int reportNumber, char* buffer, int maxLen)	;
    
  private:
    const bool	_useReportIDs;
};
    
/************************************************************************
*  class USBRelay							*
************************************************************************/
class USBRelay : public USBHid
{
  public:
		USBRelay(uint16_t idVendor=0x16c0, uint16_t idProduct=0x05df)
		    :USBHid(idVendor, idProduct, false), _state(0)
		{
		    setState();
		}

    USBRelay&	setPower(u_int channel, bool on)
		{
		    if (on)
			_state |=  (0x1 << channel);
		    else
			_state &= ~(0x1 << channel);
		    return setState();
		}
    
    bool	isPowerOn(u_int channel) const
		{
		    return _state & (0x1 << channel);
		}

  private:
    USBRelay&	setState()
		{
		    char	buffer[17];
		    std::fill(std::begin(buffer), std::end(buffer), 0);
		    buffer[1] = 0x81;
		    buffer[2] = char(_state);
		    setReport(buffer, sizeof(buffer));
		    return *this;
		}
    
  private:
    u_int	_state;

};

}
#endif	// !TU_USBPP_H
