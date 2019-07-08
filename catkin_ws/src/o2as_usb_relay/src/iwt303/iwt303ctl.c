/*
 * IWT303 Control Program
 * Copyright (C) 2014 Tokyo Devices, I.W. Technology Firm, Inc.
 * http://tokyodevices.jp/
 * License: GNU GPL v2 (see License.txt)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "hiddata.h"
#include "usbconfig.h"  /* for device VID, PID, vendor name and product name */

static char*
usbErrorMessage(int errCode)
{
    static char	buffer[80];

    switch(errCode)
    {
      case USBOPEN_ERR_ACCESS:
	return "Access to device denied";
      case USBOPEN_ERR_NOTFOUND:
	return "The specified device was not found";
      case USBOPEN_ERR_IO:
	return "Communication error with device";
      default:
	sprintf(buffer, "Unknown USB error %d", errCode);
	return buffer;
    }

    return NULL;    /* not reached */
}

static usbDevice_t*
openDevice(char* serialNumber)
{
    usbDevice_t	*	dev = NULL;
  
  // 検索するベンダID
    unsigned char	rawVid[2] = {USB_CFG_VENDOR_ID};
  
  // 検索するデバイスID
    unsigned char	rawPid[2] = {USB_CFG_DEVICE_ID};
  
  // ベンダ名
    char		vendorName[] = {USB_CFG_VENDOR_NAME, 0};
  // プロダクト名
    char		productName[] = {USB_CFG_DEVICE_NAME, 0};
  
    int			vid = rawVid[0] + 256 * rawVid[1];
    int			pid = rawPid[0] + 256 * rawPid[1];	
    int			err;
  
    if ((err = usbhidOpenDevice(&dev, vid, vendorName, pid,
				productName, serialNumber, 0)) != 0)
    {
	if ( serialNumber != NULL )
	{
	    fprintf(stderr, "error finding %s: %s\n",
		    serialNumber, usbErrorMessage(err));
	}
	return NULL;
    }
  
    return dev;
}

static void
usage(char* myName)
{
    fprintf(stderr, "IWT303 host controller Version 0.0.1\n");
    fprintf(stderr,
	    "2013-2014 (C) Tokyo Devices, I.W. Technology Firm, Inc.\n");
    fprintf(stderr, "usage:\n");
    fprintf(stderr,
	    "  %s list ... List all serial number of detected IWT303(s).\n",
	    myName);
    fprintf(stderr,
	    "  %s set <SerialNumber|\"ANY\"> <StateNumber> ... Set state of relays.\n",
	    myName);
}

int
main(int argc, char** argv)
{
    usbDevice_t*	dev;
    char		buffer[17];    /* 16バイトのバッファ+レポートID用1バイト */
    int			err;
  
    if(argc < 2)
    {
	usage(argv[0]);
	exit(1);
    }

    if (strcmp(argv[1], "list") == 0)
    {		
      // IWT303のシリアル番号のリストを1行1台で表示します		
	if ((dev = openDevice(NULL)) == NULL)
	    exit(1);
    }
    else if (strcmp(argv[1], "set") == 0)
    {
      // 指定されたシリアル番号のリレーの状態を設定する
    
	if ( argc < 4 )
	{
	    fprintf(stderr, "Invalid arguments.\n");
	    usage(argv[0]);
	    exit(1);
	}
	else
	{
	  // シリアル番号を指定してUSBデバイスを開く
	    if((dev = openDevice(argv[2])) == NULL)
		exit(1);
	      
	  // レポートを作成する
	    memset(buffer, 0, sizeof(buffer));
	    buffer[1] = 0x81;			// IWT303 - リレー状態設定命令
	    buffer[2] = (char)atoi(argv[3]);	// 8ビットの状態変数
	      
	    if((err = usbhidSetReport(dev, buffer, sizeof(buffer))) != 0)
	      /* add a dummy report ID */
		fprintf(stderr,
			"error writing data: %s\n", usbErrorMessage(err));
	}
    }
    else if (strcmp(argv[1], "init") == 0)
    {
	time_t	epoc;
	  
	memset(buffer, 0, sizeof(buffer));
	buffer[0] = 0;    // ダミーのレポートID
	buffer[1] = 0x82; // IWT303 - シリアル番号初期化命令
	  
      // エポック秒を得て、10文字の文字列に変換する
	time(&epoc);
	sprintf(&buffer[2], "%i", (int)epoc);
	  
      // 最初のIWT303デバイスを開く
	if ((dev = openDevice("ANY")) == NULL)
	    exit(1);
	  
      // 接続されているIWT303の1台のシリアル番号を初期化する
	if ((err = usbhidSetReport(dev, buffer, sizeof(buffer))) != 0) 
	    fprintf(stderr, "error init serial: %s\n", usbErrorMessage(err));
	  
	printf("Set serial number to %s\n",&buffer[2]);
    }
    else
    {
	usage(argv[0]);
	exit(1);
    }
	
  // デバイスをクローズ
    usbhidCloseDevice(dev);
	
    return 0;
}
