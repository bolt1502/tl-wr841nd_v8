/******************************************************************************
*
* Copyright (c) 2010 TP-LINK Technologies CO.,LTD.
* All rights reserved.
*
* FILE NAME  :   serveForHttpd.h
* VERSION    :   1.0
* DESCRIPTION:   communicate with httpd.
*
* AUTHOR     :   HouXB <houxubo@tp-link.net>
* CREATE DATE:   09/20/2010
*
* HISTORY    :
* 01   09/20/2010  HouXB     Create.
*
******************************************************************************/
#ifndef SERVE_FOR_HTTPD_H
#define SERVE_FOR_HTTPD_H

#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include <sys/un.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <upnp/upnp.h>
#include <time.h> 

#include "ushare.h"

#define WAIT_HTTPD_TO_CONNECT_TIMEOUT 300
#define USHARE_HTTPD_SYN_HEADER_LEN 8

#define MAX_FULL_PATH_LEN 128
#define MAX_SERVER_NAME_LEN 32
#define MAX_MANUFATURE_NAME_LEN 32
#define MAX_MANUFATURE_URL_LEN 48
#define MAX_MODEL_NAME_LEN 16

#define MAX_SHARE_NAME_LEN 16
#define MAX_PATITION_LEN 16

#define MAX_ERROR_STRING_LEN 16

#define MAX_SHARE_FOLDERS_NUM 6

#define CONNECT_TIMEOUT_TIMES 10
#define USHARE_HTTPD_SYN_HEADER_MSG "HTTPDSYN"

#define MEDIA_SERVER_RESP_OK 200

#define INVALID_CONTENT_INDEX -1

#define MEDIA_SERVER_OK 0
#define MEDIA_SERVER_ERROR -1

/* 1Hour = 3600Sec */
#define SECONDS_PER_HOUR 3600

/* one content in ushare's content list by HouXB, 19Sep10 */
typedef struct{
	int index;		
	char displayName[MAX_SHARE_NAME_LEN];
	char fullPath[MAX_FULL_PATH_LEN];
}CONTENT_INFO;

/* server name and status. by HouXB, 19Sep10 */
typedef struct{
	int serverStatus;		/* 0, stop; 1, start */
	char servName[MAX_SERVER_NAME_LEN];
	char manufatureName[MAX_MANUFATURE_NAME_LEN];
	char manufatureUrl[MAX_MANUFATURE_URL_LEN];
	char modelName[MAX_MODEL_NAME_LEN];
	int isAutoScan;	/* 0, No; 1, Yes */
	int scanInterval; /* unit is hour */
}SERVER_INFO;



/* folders info for web page to display by HouXB, 19Sep10 */
typedef struct{
	CONTENT_INFO contentInfo;	
	char partition[MAX_PATITION_LEN];
	int scanFlag;
	int deleteFlag;
}FOLDER_INFO;

/* All infos that be used on web page. by HouXB, 20Sep10 */
typedef struct{
	int validFolderNum;
	FOLDER_INFO folderListInfo[MAX_SHARE_FOLDERS_NUM];
}FOLDER_INFO_LIST;

/* all info show in web by HouXB, 20Sep10 */
typedef struct{
	SERVER_INFO serverInfo;
	FOLDER_INFO_LIST folderList;
}INIT_SERVER_INFO;

/* auto scan parameter. by HouXB, 14Oct10 */
typedef struct
{
	int isAutoScan;
	int autoScanInterval;
}AUTO_SCAN_PARA;

/* httpd send req cmd type, by HouXB, 19Sep10 */
typedef enum{
	INVALID_CMD,
	START_MEDIA_SERVER,
	STOP_MEDIA_SERVER,
	ADD_FOLDER,
	DELETE_FOLDER,
	REFRESH_CONTENT_LIST,
	GET_CONTENT_LIST,
	SET_AUTO_SCAN
}HTTPD2MEDIASERVER_CMD_TYPE;

/* cmd info send from httpd to ushare. by HouXB, 19Sep10 */
typedef struct{
	HTTPD2MEDIASERVER_CMD_TYPE cmdType;
	CONTENT_INFO contentInfo;
	AUTO_SCAN_PARA autoScanPara;
}REQ_INFO;

/* ushare respone info to httpd by HouXB, 20Sep10 */
typedef struct{
	HTTPD2MEDIASERVER_CMD_TYPE cmdType;
	int respCode;
	CONTENT_INFO contentList[MAX_SHARE_FOLDERS_NUM];
}RESP_INFO;

typedef struct{
	int scanFlag;
	int orgInterval;
	int delayH;
	int delayS;
}AUTO_SCAN_DELAY;

extern INIT_SERVER_INFO gUshareInitInfo;

/******************************************************************************
* FUNCTION      : setUshareInitInfo()
* AUTHOR        : HouXB <houxubo@tp-link.net>
* DESCRIPTION   : accept httpd connect and set init info needed by start ushare 
* INPUT         : sd, listen socket description
*
* OUTPUT        : N/A
* RETURN        : N/A
* OTHERS        :
******************************************************************************/
int setUshareInitInfo(int sd);

/******************************************************************************
* FUNCTION      : serverProcessInit()
* AUTHOR        : HouXB <houxubo@tp-link.net>
* DESCRIPTION   : server process for httpd 
* INPUT         : pointer sd, address which should be used to save socket sd
*
* OUTPUT        : sd, estiblished socket sd
* RETURN        : 0, ERROR; 1, SUCCESS
* OTHERS        :
******************************************************************************/
int serverProcessInit(int *sd);

/******************************************************************************
* FUNCTION      : recvSocketData()
* AUTHOR        : HouXB <houxubo@tp-link.net>
* DESCRIPTION   : receive data from socket which identify by SD 
* INPUT         : 	int sd, 给定socket描述符
				char *buf, 写入数据地址
				int maxLen, 最大接收数据长度
				int timeOutSec, 等待时间的秒数
				int timeOutUsec,等待时间的微妙数
*
* OUTPUT        : N/A
* RETURN        : 接收的数据长度，-1 表示出错
* OTHERS        :
******************************************************************************/
int recvSocketData(int sd, char *buf, int maxLen, int timeOutSec, int timeOutUsec);

/******************************************************************************
* FUNCTION      : processHttpdReq()
* AUTHOR        : HouXB <houxubo@tp-link.net>
* DESCRIPTION   : get req from httpd and give the response back 
* INPUT         : clientSd, socket descriptor used in send/recv msg
*
* OUTPUT        : N/A
* RETURN        : N/A
* OTHERS        :
******************************************************************************/
void processHttpdReq(int clientSd);

/******************************************************************************
* FUNCTION      	: rebuildMetaList()
* AUTHOR        	: HouXB <houxubo@tp-link.net>
* DESCRIPTION  	: rebuild meta list 
* INPUT			: 
*
* OUTPUT        	: 
* RETURN        	: 
* OTHERS        	: 
******************************************************************************/
void rebuildMetaList();

/******************************************************************************
* FUNCTION      	: autoScan()
* AUTHOR        	: HouXB <houxubo@tp-link.net>
* DESCRIPTION  	: auto scan specify folders to build meta list 
* INPUT			: 
*
* OUTPUT        	: 
* RETURN        	: 
* OTHERS        	: 
******************************************************************************/
void autoScan();

#endif

