/******************************************************************************
*
* Copyright (c) 2010 TP-LINK Technologies CO.,LTD.
* All rights reserved.
*
* FILE NAME  :   serveForHttpd.c
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

#include "serveForHttpd.h"

INIT_SERVER_INFO gUshareInitInfo;
AUTO_SCAN_DELAY gAutoScanDelay;	
timer_t gTid;

typedef void (* sighandler)();

static timer_t tpCreateTimer(int signum, sighandler handler_func)
{
	timer_t tid;
	struct sigevent se;

	memset (&se, 0, sizeof (se));
	
	signal(signum, handler_func);
	
	se.sigev_notify = SIGEV_SIGNAL;
	se.sigev_signo = signum;
	se.sigev_notify_function = handler_func;	
	
	if (timer_create(CLOCK_MONOTONIC, &se, &tid) < 0)
	{ 	
		perror("timer_creat");
		return -3; 
	}

	return tid;
}

static int tpSetTimer(timer_t timer_id, int firstRun, int interval)
{
	struct itimerspec ts, ots;
	
	ts.it_value.tv_sec = firstRun;
	ts.it_value.tv_nsec =  0;
	ts.it_interval.tv_sec = interval;
	ts.it_interval.tv_nsec = 0;

	/* we need a firstRun delay, so TIMER_ABSTIME cant be set -- lsz 081215 
	 *if (timer_settime(timer_id, TIMER_ABSTIME, &ts, &ots) < 0) */
	 
	if (timer_settime(timer_id, 0, &ts, &ots) < 0)
	{
		perror ("tpSetTimer");
		return (-1);
	}

	return (0);
}

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
int setUshareInitInfo(int sd){

	int timeout = 0;
	int clientSd = -1;	
	int len = 0;
	int ret = -1;
	char buf[USHARE_HTTPD_SYN_HEADER_LEN + 1] = {' '};

	struct sockaddr_un cli;
	
	while( (timeout < WAIT_HTTPD_TO_CONNECT_TIMEOUT)
			&& (clientSd == -1) )
	{
		
		clientSd = accept(sd,(struct sockaddr *)&cli,&len);
		
		if(-1 != clientSd){
		 	
			ret = recvSocketData(clientSd, buf, USHARE_HTTPD_SYN_HEADER_LEN, 1, 0);
			if(USHARE_HTTPD_SYN_HEADER_LEN != ret){
				printf("synchronize error!\n");
				return 0;
			}
			printf("recv synchronization %s\n",buf);
			
			memset((char*)&gUshareInitInfo, 0, sizeof(INIT_SERVER_INFO));
			if(!strncmp(buf,USHARE_HTTPD_SYN_HEADER_MSG, USHARE_HTTPD_SYN_HEADER_LEN)){
				
				ret = recvSocketData(clientSd, (char*)&gUshareInitInfo, 
					sizeof(INIT_SERVER_INFO), 5, 0);
				
				if(sizeof(gUshareInitInfo) != ret){
					printf("get ushare init info error!\nsizeof(gUshareInitInfo) is:%d, ret is:%d\n",
						sizeof(gUshareInitInfo), ret);
					return -1;
				}
				
				/*
				printf("\n\n\n****\ninit info,len %d\nserver name:%s\nservice status:%d\n \
						validFolderNum:%d\nauto scan flag:%d\nscan interval:%d\n \
						manufature %s\n manufature url %s\n****\n\n\n",
					ret,
					gUshareInitInfo.serverInfo.servName,
					gUshareInitInfo.serverInfo.serverStatus,
					gUshareInitInfo.folderList.validFolderNum,
					gUshareInitInfo.serverInfo.isAutoScan,
					gUshareInitInfo.serverInfo.scanInterval,
					gUshareInitInfo.serverInfo.manufatureName,
					gUshareInitInfo.serverInfo.manufatureUrl);
			 	*/ 

				/* init auto scan parameters. by HouXB, 14Oct10 */
				gAutoScanDelay.scanFlag = gUshareInitInfo.serverInfo.isAutoScan;
				gAutoScanDelay.orgInterval = gUshareInitInfo.serverInfo.scanInterval;
				gAutoScanDelay.delayH =  gAutoScanDelay.orgInterval -1;
				gAutoScanDelay.delayS = SECONDS_PER_HOUR;
			}		
		}
		
		timeout++;
		sleep(1);
	}	

	gTid = tpCreateTimer(SIGRTMIN, autoScan);

	/* start timer anyway */
	tpSetTimer(gTid, 10, SECONDS_PER_HOUR/10);
		
	return clientSd;	
}

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
int serverProcessInit(int *sd){	
	int ret = -1;
	int onOff = 1;
	
	struct sockaddr_un addr;
	*sd = socket(AF_LOCAL, SOCK_STREAM, 0);	
	
	if (*sd < 0) {
		printf("ushare server create sd error\n");
		return 0; 
	}
	
	if (fcntl(*sd, F_SETFL, O_NONBLOCK) != 0) {
		printf("ushare server fcntl set non block error\n");
		return 0;               
	}	

	if (setsockopt(*sd, SOL_SOCKET, SO_REUSEADDR,
	               &onOff, sizeof(onOff))) {
		printf("ushare server reuse tcp socket Address error\n");
		return 0;               
	}	
	memset(&addr, 0, sizeof(addr));
	
	unlink("/var/run/mediaServerUshare");
	addr.sun_family = AF_LOCAL;	
	strncpy(addr.sun_path, "/var/run/mediaServerUshare", sizeof(addr.sun_path) - 1);
	
	if (bind(*sd, (struct sockaddr *)&addr, SUN_LEN(&addr))) 
	{
		printf("ushare server bind error\n");
		return 0;                
	}

	/* printf("ushare server begin Listening!\n"); */
	ret = listen(*sd,1);	
	if(-1 == ret){
		printf("ushare server listen error!\n");
		return 0;
	}
	return 1;
}

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
int recvSocketData(int sd, char *buf, int maxLen, int timeOutSec, int timeOutUsec){
	struct timeval tv;
	fd_set rfds;
	int ret = -1;
	int res;

	tv.tv_sec = timeOutSec;
	tv.tv_usec = timeOutUsec;
	
	FD_ZERO(&rfds);
	FD_SET(sd, &rfds);
	res = select(sd + 1, &rfds, NULL, NULL, &tv);
	if(res <= 0){
		return ret;
	}
	
	if (FD_ISSET(sd, &rfds)) {		
		ret = recv(sd, buf, maxLen, 0);
		/* printf("in ushare recv len:%d; buf:\n%s\n",ret,buf); */
	}
	return ret;
}

int sendRespToHttpd(int clientSd, RESP_INFO *respInfo)
{
	int ret = -1;
	if((ret = send(clientSd, (char*)respInfo, sizeof(RESP_INFO), 0)) < 0){
		printf("send error");
		return MEDIA_SERVER_ERROR;
	}

	/* 
	printf("\n*****\nmedia server send resp\ncmd type:%d\nmsg len:%d\n*****\n",respInfo->cmdType,ret);
	*/
	return MEDIA_SERVER_OK;
}

int sendOKRespToHttpd(int clientSd, RESP_INFO *respInfo, HTTPD2MEDIASERVER_CMD_TYPE cmdType)
{
	respInfo->cmdType = cmdType;
	respInfo->respCode = MEDIA_SERVER_RESP_OK;
	sendRespToHttpd(clientSd, respInfo);
	return MEDIA_SERVER_OK;
}

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
void processHttpdReq(int clientSd)
{	
	REQ_INFO reqInfo;
	RESP_INFO respInfo;
	int recvLen;
	int i = 0;
	
	while(1){
		reqInfo.cmdType = INVALID_CMD;
		recvLen = recvSocketData(clientSd, (char*)&reqInfo, sizeof(REQ_INFO), 5, 0);
		
		
		if(-1 == recvLen){
			//autoScan();
			continue;
		}
		else if(0 == recvLen){
			/*  
			printf("\nmedia server recv req\ncmd type is:%d\nreq len is:%d \n so httpd is crashed!!\n\n",
				reqInfo.cmdType, recvLen);
			*/
			sleep(3);			
			break;
		}

		/*  
		printf("\nmedia server recv req\ncmd type is:%d\nreq len is:%d\n\n",
			reqInfo.cmdType, recvLen);*/
		
		switch(reqInfo.cmdType){
			case START_MEDIA_SERVER:
				if(0 == UpnpGetStatus())
				{
					UpnpEnable();
					UpnpSendAdvertisement (ut->dev, 1800);
				}
				sendOKRespToHttpd(clientSd, &respInfo, reqInfo.cmdType);
				printf("start media server...\n");
			break;
			
			case STOP_MEDIA_SERVER:
				if(1 == UpnpGetStatus())
				{
					UpnpDisable();
				}
				sendOKRespToHttpd(clientSd, &respInfo, reqInfo.cmdType);
				
				printf("stop media server...\n");
			break;
			
			case ADD_FOLDER:

				/* send resp and then scan folder to make httpd response quickly. by HouXB, 01Mar12 */
				sendOKRespToHttpd(clientSd, &respInfo, reqInfo.cmdType);

				if (reqInfo.contentInfo.fullPath && 
					reqInfo.contentInfo.displayName)
				{
					ut->contentlist = content_add(ut->contentlist, 
											reqInfo.contentInfo.fullPath,
											reqInfo.contentInfo.displayName);
					rebuildMetaList();
				}				
				printf("add folder over\n");
			break;
			
			case DELETE_FOLDER:
				{
					int oldCount = ut->contentlist->count;
					ut->contentlist = content_del(ut->contentlist, reqInfo.contentInfo.index);

					if (oldCount != ut->contentlist->count)
						rebuildMetaList();
					
					sendOKRespToHttpd(clientSd, &respInfo, reqInfo.cmdType);
					printf("delete folder...\n");
				}
			break;
			
			case REFRESH_CONTENT_LIST:
				rebuildMetaList();			
				sendOKRespToHttpd(clientSd, &respInfo, reqInfo.cmdType);
				
				printf("refresh content list...\n");
			break;
			
			case GET_CONTENT_LIST:
				if(ut !=NULL && ut->contentlist !=NULL)
				{
					for (i=0; (i<ut->contentlist->count)&&(i<MAX_SHARE_FOLDERS_NUM); i++)
					{
						respInfo.contentList[i].index = i;
						
						sprintf(respInfo.contentList[i].displayName,"%s", 
								ut->contentlist->displayName[i]);
						sprintf(respInfo.contentList[i].fullPath,"%s", 
								ut->contentlist->content[i]);
						/*  
						printf("index:%d; name:%s; path:%s\n", 
							i,
							ut->contentlist->displayName,
							ut->contentlist->content);*/
					}
					if(i<MAX_SHARE_FOLDERS_NUM)
					{
						respInfo.contentList[i].index = INVALID_CONTENT_INDEX;
					}
				}
				else
				{
					respInfo.contentList[0].index = INVALID_CONTENT_INDEX;
					sprintf(respInfo.contentList[0].fullPath,"%s", "/tmp");
				}				
				sendOKRespToHttpd(clientSd, &respInfo, reqInfo.cmdType);

				/* 
				printf("ushare get content list send respInfo:\n");
				for (i=0; i<MAX_SHARE_FOLDERS_NUM; i++)
				{
					printf("index:%d; name:%s; path:%s\n", 
							respInfo.contentList[i].index,
							respInfo.contentList[i].displayName,
							respInfo.contentList[i].fullPath);
				} 
				*/

			break;

			case SET_AUTO_SCAN:
				gAutoScanDelay.scanFlag = reqInfo.autoScanPara.isAutoScan;

				if (1 == gAutoScanDelay.scanFlag)
				{
					gAutoScanDelay.orgInterval = reqInfo.autoScanPara.autoScanInterval;
					gAutoScanDelay.delayH = gAutoScanDelay.orgInterval -1;
					gAutoScanDelay.delayS = SECONDS_PER_HOUR;
				}
				
				sendOKRespToHttpd(clientSd, &respInfo, reqInfo.cmdType);
				
			break;
			
			default:
			break;
		}

	}
}

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
void rebuildMetaList()
{
	if (ut->contentlist)
	{
		rebuild_list_lock();
		free_metadata_list (ut);
		build_metadata_list (ut);
		rebuild_list_unlock();
	}	
}

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
void autoScan()
{
	/* only auto scan when scanFlag was set. by HouXB, 14Oct10 */
	if(gAutoScanDelay.scanFlag)
	{		
		if (gAutoScanDelay.delayS > 0)
		{
			gAutoScanDelay.delayS -= SECONDS_PER_HOUR/10;			
		}
		else if(gAutoScanDelay.delayH > 0)
		{
			gAutoScanDelay.delayH -= 1;
			gAutoScanDelay.delayS += SECONDS_PER_HOUR;
		}
		else
		{			
			if (1 == UpnpGetStatus())
			{
				rebuildMetaList();
			}
			gAutoScanDelay.delayH = gAutoScanDelay.orgInterval - 1;
			gAutoScanDelay.delayS = SECONDS_PER_HOUR;
		}
		/* auto scan info, it is easier for test,
		 * it will be printed out in every 6 mins  */
		printf("time up, auto scan..\n");
	}
}

