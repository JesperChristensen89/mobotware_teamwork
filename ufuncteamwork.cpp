/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
 *   See svn log for modification history                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef OPENCV2
#include <legacy/compat.hpp>
#endif

#include <urob4/usmltag.h>
#include <cstdlib>
#include <boost/concept_check.hpp>

#include "ufuncteamwork.h"
#include "teamwork.h"
#include "gmk.h"
#include "uart.h"
#include "commands.h"
#include "tcp.h"
#include <stdio.h>
#include <stdlib.h>
#include "../aupoly/urespoly.h"
#include <urob4/uresposehist.h>
//#include <urob4/ucmdexe.h>
//#include <../libs/eigen3/Eigen/src/Eigen2Support/Geometry/Translation.h>
#include <highgui.h>
#include <time.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <pthread.h>
#include <unistd.h>
#include <cmath>
 
TeamWork teamwork;
GMK gmk;
UART uart;
TCP tcpClient;
FILE * logFile;
bool startLogging = false;

double distanceToGmk = 0;
double angleToGmk = 0;
double vel = 0;
double turnRadius = 0.0;
bool wifiIsConnected = false;
bool clrVelReg = false;

bool interupt = false;
double angleOveride = 0;
double distanceOveride = 0.40;
int overideState = 0;

double logVel = 0;
double logAngle = 0;
double masterVel = 0;

int loopcounter = 0;

// defines the ID the robot is looking for
int ID2find = 1;
// true if robot sees left side of robot [default = true]
bool leftSide = true;
// flag to start teensy log
bool startTeensyLog = false;
// flag to stop visionLog
bool stopVisionLog = false;
// indicates if the leader got the msg
bool leaderProceeding = false;

void* uartReceiver(void* arg)
{
  while(1)
  {
    uart.receive();
    usleep(500);
  }
  
  pthread_exit(NULL);
}

void* tcpReceiver(void* arg)
{
  while(1)
  {
    if (wifiIsConnected)
    {
      tcpClient.receive();
      usleep(500);
    }
  }
  
  pthread_exit(NULL);
}

UFuncTeamWork::~UFuncTeamWork(){}

  
  /**
  Handle incomming command
  Must return true if the function is handled -
  otherwise the client will get a 'failed' reply */
bool UFuncTeamWork::handleCommand(UServerInMsg* msg, void* extra)
{ // handle command(s) send to this plug-in
  const int MRL = 2000;
  char reply[MRL];
  bool ask4help;
  const int MVL = 50;
  char val[MVL];
  int camDevice = -1;
  bool imgPoolIsSet = false;
  int imgPoolNum = -1;
  bool stopRegbot;
  UImage * img = (UImage *)extra; 
  UImage * gmkImg;
  USmlTag tag;
  UCamPush * cam = NULL;
  bool result;
  bool debug = true; // default is debug on
  bool smrcl = false; // default is <ball ...> reply
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  stopRegbot = msg->tag.getAttValue("stop",val,MVL);
  bool setup = false;
  bool regbotreadytogo = false;
  if (not ask4help)
  { // get all other parameters
    msg->tag.getAttValueInt("device", &camDevice);
    imgPoolIsSet = msg->tag.getAttValueInt("img", &imgPoolNum);
    msg->tag.getAttValueBool("debug", &debug, true);
    msg->tag.getAttValueBool("smrcl", &smrcl, true);
    msg->tag.getAttValueBool("setup", &setup, true);
    msg->tag.getAttValueBool("ready", &regbotreadytogo, true);
    msg->tag.getAttValueInt("M",&missionState);
  }
  if (setup)
  {
    getCorePointer()->postCommand(-36,"varpush struct=gmk call=\"teamwork.getGMK(0)\"\n");
    getCorePointer()->postCommand(-36,"poolpush img=13 i=1 cmd=\"teamwork img=13\"\n");
    
    uart.init();
    
    pthread_t uartReceiveThread;
    int ret = pthread_create(&uartReceiveThread, NULL, &uartReceiver, NULL);
    if (ret != 0)
    {
      printf("Error: pthread_create() failed\n");
      exit(EXIT_FAILURE);
    }
    
    pthread_t tcpReceiveThread;
    ret = pthread_create(&tcpReceiveThread, NULL, &tcpReceiver, NULL);
    if (ret != 0)
    {
      printf("Error: pthread_create() failed\n");
      exit(EXIT_FAILURE);
    }
    
    uart.send((char*)"S=1\n");
    
    logFile = fopen("../raspilog.txt", "w");
    
    
    
    printf("\nSetup completed!\n");
    
    
    result = true;
  }
  if(regbotreadytogo)
  {
    tcpClient.sendData("readyToGo\n");
    result = true;
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart("TeamWork");
    sendText("--- available TeamWork options\n");
    sendText("img=X             Get image from image pool - else take new image\n");
    sendText("setPush           Initiates the push commands required by this app\n");
    sendText("smrcl             Format the reply for MRC (<vision vis1=\"x.x\" vis2=\"y.y\" .../>\n");
    sendText("help              This message\n");
    sendText("See also: var teamwork for other parameters and results\n");
    sendHelpDone();
    sendInfo("done");
    result = true;
    
  }
  
  else if (stopRegbot)
  {
    uart.send((char*)"M=3\n");
    uart.send((char *)"A=998\n");
    startLogging = false;
    fclose(logFile);
    printf("Sending stop command\n");
    
    stopVisionLog = true;
  }
  else
  { 
    
      
      if (imgPoolIsSet)
      { 
	// take image from image pool
	img = imgPool->getImage(imgPoolNum, false);
	result = (img != NULL);
	if (result and camDevice < 0)
	  //take image device from image
	  camDevice = img->camDevice;
	if (result)
	{
	  cam = camPool->getCam(camDevice);
	  result = (cam != NULL);
	}
      }
      
      else if (img != NULL)
    { 
      // we have an image, so just camera is needed
      camDevice = img->camDevice;
      cam = camPool->getCam(camDevice);
      result = (cam != NULL);
    }
    
    else
    { 
      // get new image from a camera and store in first imagepool number for ball images
      img = imgPool->getImage(varPoolImg->getInt(), true);
      result = getCamAndRawImage(&cam,        // result camera           out
				&img,        // result image            out
				&camDevice,  // camera device number    in-out
				NULL,        // pushed image (YUV)      in
				"", camDevice + 3);         // camera position name    in
      if (result)
	result = (img != NULL and cam != NULL);
    }
    
    // camera and image is now available
    // time to kick some ass
    
    if (result)
    { 
      
      
      
      // make sure client is connected to server
      if (not wifiIsConnected)
      {
	wifiIsConnected = tcpClient.conn("10.0.1.159",1336);
	if (not wifiIsConnected)
	  return false;
      }
         
      loopcounter += 1;
         
         
      if (interupt)
      {
	switch (overideState)
	{
	  case 0:
	    
	    printf("Sending overide angle\n");
	   
	    char overrideAngleStr[20];
	    snprintf(overrideAngleStr,20, "OA=%f\n", angleOveride);
	    uart.send(overrideAngleStr);
	    
	    angleToGmk += (angleOveride*180/M_PI);
	    
	    overideState = 1;
	    break;
	    
	  case 1:
	    if(regbotReady)
	    {
	      regbotReady = false;
	      
	      printf("Sending overide distance\n");
	    
	      char overrideDistStr[20];
	      snprintf(overrideDistStr,20, "OD=%f\n", distanceOveride+0.15);
	      uart.send(overrideDistStr);
	      
	      overideState = 2;
	    }
	    
	    break;
	    
	    
	  case 2:
	    if(regbotReady)
	    {
	      regbotReady = false;
	      printf("Sending overide mission\n");
	      
	      if (angleOveride == 3.14)
	      {
		uart.send((char*)"M=10\n");
		//startLogging = true;
		angleToGmk = 0.0;
	      }
	      else
		uart.send((char*)"M=8\n");
	      
	      overideState = 3;
	    }
	    break;
	    
	  case 3:
	    if (regbotReady)
	    {
	      distanceOveride = 0.4;
	      tcpClient.sendData("readyToGo\n");
	      interupt = false;
	    }
	    break;
	    
	  default:
	    break;
	}
	
	return true;
      }
	
	if (startTeensyLog)
	{
	  switch (overideState)
	  {
	    case 0:
	      uart.send((char*)"M=97\n"); // start teensy log
	      overideState = 1;
	      break;
	    case 1:
	      if(regbotReady)
	      {
		regbotReady = false;
		startTeensyLog = false;
		
		overideState = 0;
	      }
	      break;
	  }
	  return true;
	}
		
            
     
      // there is an image, make the required analysis
      
      cv::Mat imgCV = cv::cvarrToMat(img->cvArr()); 

      switch (missionState)
      {
	case 0:
	  if (wifiIsConnected)
	  {
	    tcpClient.sendData("start\n");
	    printf("Starting guidemark search\n");
	    missionState = 1;
	  }
	  break;
	
	case 1: // get guidemarks
	  if (not gmk.isRunning() and tcpClient.regbotStarted)
	  {
	    //tcpClient.sendData("readyCmd\n");
	    uart.send((char*)"Y=1\n");
	    gmkImg = imgPool->getImage(55,true);
	    gmkImg->copy(img);
	    gmkImg->setName("GMK image");
	    gmkImg->updated();
	    gmk.setReadyFlag(false);
	    findGMK();
	    gmkTries += 1;
	    missionState = 2;
	  }
	  break;
	  
	case 2: // verify CRC and get distance
	  if (gmk.getReadyFlag())
	  {
	    bool IDavailable = gmk.findID(ID2find);
	    	    
	    if (IDavailable)
	    {
	      bool crcOK = verifyGMK(ID2find);
	      
	      if (crcOK)
	      {
		
		// check for left or right side
		char IDstr[10];
		snprintf(IDstr, 10, "%d", ID2find);
	
		if ( IDstr[strlen(IDstr)-1] == '0')
		{
		  printf("Venstre side!\n");
		  leftSide = true;
		}
		else           
		{
		  leftSide = false;
		  printf("Højre side!\n");
		}
		
		for (int i=0; i < 6; i++)
		{
		  char gmkPoseCall[50];
		  snprintf(gmkPoseCall,50,"gmk.gmk%d.gmkPose[%d]",ID2find,i);
		  
		  const char* gmkPoseCallConst = gmkPoseCall;
		  
		  getGlobalValue(gmkPoseCallConst,&gmkPose[i]);
		
		}
	
		printf("Distance to GMK is: x=%f y=%f z=%f\nRotation is: Omega=%f Phi=%f Kappa=%f\n",gmkPose[0],gmkPose[1],gmkPose[2],gmkPose[3]*180/3.14,gmkPose[4]*180/3.14,gmkPose[5]*180/3.14);
		
		missionState = 3;
			
		
	      }
		
	      else
	      {
		if (gmkTries == 2)
		  missionState = 21;
		else
		  missionState = 1;
	      }
	    }
	    else
	    {
	      if (gmkTries == 2)
		missionState = 21;
	      else
		missionState = 1;
	    }
	  }
	  break;
	  
	case 21: // case 2.1 turns the robot just a bit to make a new gmk search
	  
	  printf("Turning a bit\n");
	  
	  angleToGmk += M_PI/10;
	  
	  if (angleToGmk >= 2*M_PI)
	    angleToGmk -= 2*M_PI;
	  else if (angleToGmk <= -2*M_PI)
	      angleToGmk += 2*M_PI;
	  
	  char angleAdjStr[50];
	  snprintf(angleAdjStr,50,"A=%.3f\n",angleToGmk);
	  uart.send(angleAdjStr);
	  
	  gmkTries = 0;
	  
	  missionState = 22;
	  
	  break;
	  
	case 22: // case 2.2 waits for direction on regbot to get adjusted
	  if ( regbotReady)
	  {
	    regbotReady = false;
	    
	    missionState = 1;
	  }
	  break;
	 
	case 3: // align and drive in y
	 
	  printf("Calculating route\n");
	  
	  if (leftSide)
	    distanceToGmk = 0.40 - (cos(gmkPose[5]-M_PI/2)*gmkPose[0] + gmkPose[1]); // y direction drive
	  else
	    distanceToGmk = 0.45 + (cos(gmkPose[5]-M_PI/2)*gmkPose[0] - gmkPose[1]);

	  angleToGmk = angleToGmk + gmkPose[5] + M_PI/2;// - M_PI/3; // align with target
	  
	  if (not leftSide)
	    angleToGmk -= M_PI;
	  
	  turnRadius = (sin(gmkPose[5]-M_PI/2) * gmkPose[0])/2;
	  
	  if (angleToGmk >= 2*M_PI)
	    angleToGmk -= 2*M_PI;
	  else if (angleToGmk <= -2*M_PI)
	      angleToGmk += 2*M_PI;
	  
	  /*
	  char angleStr[50];
	  snprintf(angleStr,50,"VA=%.3f%.3f\n",vel,angleToGmk);
	  uart.send(angleStr);
	  */
	  
	  char angleStr[50];
	  snprintf(angleStr,50,"A=%.3f\n",angleToGmk);
	  uart.send(angleStr);
	  
	  
	  
	  missionState = 4;
	  
	 
	  break;
	  
	case 4:
	  if (regbotReady)
	  {
	    regbotReady = false;
	  
	    printf("Sending distance\n");
	    char distanceStrY[50];
	    snprintf(distanceStrY,50,"D=%f\n",distanceToGmk);
	    uart.send(distanceStrY);
	    
	    
	    missionState = 5;
	  }
	  break;
	  
	case 5:
	  if (regbotReady)
	  {
	    regbotReady = false;
	    
	    printf("Sending mission 4\n");
	   
	   
	    uart.send((char*)"M=4\n");
	    
	    
	    missionState = 6;
	  }
	  break;
	  
	case 6: 
	  if (regbotReady) 
	  {
	    regbotReady = false;
	    
	    printf("Starting driving\n");
	    
	    missionState = 7;
	  }
	  break;
	  
	case 7: // drive in line to target
	  if (regbotReady)
	  {
	    regbotReady = false;
	    
	    if (leftSide)
	    {
	      printf("Sending turnradius\n");
	      
	      char turnStr[50];
	      snprintf(turnStr,50,"T=%f\n",turnRadius);
	      uart.send(turnStr);
	      missionState = 8;
	    }
	    else
	    {
	      printf("Sending new angle\n");
	      
	      angleToGmk -= M_PI/2;
	      
	      char angleStr[50];
	      snprintf(angleStr, 50, "A=%f\n",angleToGmk);
	      uart.send(angleStr);
	      missionState = 71;
	    }
	      
	    
	  }
	  
	  break;
	  
	case 71: 
	  if (regbotReady)
	  {
	    regbotReady = false;
	    
	    printf("Sending distance\n");
	    
	    char distanceStrX[50];
	    snprintf(distanceStrX, 50, "D=%f\n", (turnRadius*2)+0.05);
	    uart.send(distanceStrX);
	    
	    missionState = 8;
	  }
	  break;
	  
	case 8:
	  if (regbotReady)
	  {
	    regbotReady = false;
	    
	    if(leftSide)
	    {
	      printf("Sending mission 6\n");
	      uart.send((char*)"M=6\n");
	    }
	    else
	    {
	      printf("Sending mission 4\n");
	      uart.send((char*)"M=4\n");
	    }
	    
	    missionState = 9;
	  }
	  break;
	  
	
	    
	    
	    
	  
	case 9: // wait for finish driving
	  if (regbotReady)
	  {
	    regbotReady = false;
	    
	    printf("Starting driving\n");
	    
	    if (leftSide)
	      missionState = 10;
	    else
	      missionState = 91;
	  }
	  break;
	  
	case 91:
	  if ( regbotReady)
	  {
	    regbotReady = false;
	    
	    printf("Sending new angle\n");
	    
	    angleToGmk -= M_PI/2;
	      
	    char angleStr[50];
	    snprintf(angleStr, 50, "A=%f\n",angleToGmk);
	    uart.send(angleStr);
	    missionState = 10;
	    
	    loopcounter = 0;
	  }
	  break;
	  
	case 10:
	  if (regbotReady and loopcounter > 15)
	  {
	    regbotReady = false;
	    printf("Ready to go\n");
	    uart.send((char*)"M=98\n"); // clear pose 
	    
	    tcpClient.sendData("readyToGo\n");
	    
	    loopcounter = 0;
	    
	    missionState = 11;
	    
	  }
	  break;
	 
	case 11:
	  if (regbotReady and leaderProceeding)
	  {
	    leaderProceeding = false;
	    regbotReady = false;
	    angleToGmk = 0.0;
	    missionState = 12;
	    
	  }
	  else if (loopcounter > 15)
	  {
	    missionState = 10;
	    regbotReady = true;
	    wifiIsConnected = false;
	    printf("No answer from Leader - trying again!\n");
	  }
	    
	    
	  break;
	  
	case 12:
	  {
	    missionState = 13;
	    angleToGmk = 0.0;
	    
	    startLogging = true;
	  }
	  break;
	  
	case 13:
	  teamwork.doWork(imgCV, uart);
	  if (startLogging)
	    fprintf(logFile, "%f %f %f\n", distanceOveride, logVel, logAngle);
	  break;
	  
	default:
	  break;
      
	
      }
	
	
	
	 
      
    }
    else
    {
      snprintf(reply, MRL, "failed, got image %s, got camera %d %s\n",
	      bool2str(img != NULL), camDevice, bool2str(cam != NULL));
      sendWarning(msg, reply);
    }
  }


  // return true if the function is handled with a positive result
  return result;
}
  
  
bool UFuncTeamWork::methodCall(const char* name, const char* paramOrder, char** strings, const double* pars, double* value, UDataBase** returnStruct, int* returnStructCnt)
{
  
  gmk.setReadyFlag(true);
  gmk.setRunning(false);
  
  bool result = true;
  //int i;
  UPosition pos;
  //UVariable * var;
  //bool isOK;
  // evaluate standard functions
  if ((strcasecmp(name, "getGMK") == 0))
  {
    double countID = 0;
    getGlobalValue("gmk.count",&countID);

    gmk.clearIDs();
    
    if (countID != 0)
    {

      int IDs = (int)(countID+0.5);
      
      
      
      for (int i=0; i < IDs; i++)
      {
	char gmkIdCall[50];
	snprintf(gmkIdCall,50,"gmk.IDs[%d]",i);
	
	
	double id;
	getGlobalValue(gmkIdCall,&id);
	
	int idINT = (int)(id+0.5);
	
	
	
	if (idINT != 0)
	  gmk.addID(idINT);

	  
      }
      
      
      
      
    }
    else 
    {
      //printf("No guidemarks found\n");
    }
    
    
      // it is good praktice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }

  else
    // call name is unknown
    result = false;
  return result;
}

/*
void* gmkWaitThread(void *arg )
{
  
  
  usleep(2000000);
  printf("gmkWaitThread has finished\n");
  
  gmk.setRunning(false);
  
  gmk.setReadyFlag(true);
  
  
  pthread_exit(NULL);
}
*/
  
  
void UFuncTeamWork::findGMK()
{
  /*
  pthread_t t1;
  int ret = pthread_create(&t1, NULL, &gmkWaitThread, NULL);
  if (ret != 0)
  {
    
    printf("Error: pthread_create() failed\n");
    exit(EXIT_FAILURE);
  }
  else
    */
  {
    gmk.setRunning(true);
    getCorePointer()->postCommand(-36,"gmk img=55 block=0.00571");
    printf("gmk-command posted\n");
  }
}

bool UFuncTeamWork::verifyGMK(int ID)
{
  
  double crcOK = 0;
  char getCrcOK[50];
  snprintf(getCrcOK,50,"gmk.gmk%d.crcOK",ID);
  
  const char* finalGetCrcOK = getCrcOK;
  
  getGlobalValue(finalGetCrcOK,&crcOK);
  
  printf("Crc check: %f\n",crcOK);
  
  if (crcOK == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
  
}





  /**
  Make the variables that will be available to other plugins */
void UFuncTeamWork::createBaseVar()
{
    varPoolImg  = addVar("poolImg", 45.0, "d", "(r/w) first image pool number to use");
    //varLeft     = addVar("left", 0.0, "b", "turn direction");
    //varWhite    = addVar("white", 0.0, "b", "Line color to follow");
    //varErrThresh= addVar("errThresh", 4.0, "i", "Errors to make before exiting");
    addMethod("getGMK", "d", "Get GMK");

  }
  
#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncTeamWork' with your classname, as used in the headerfile */
  return new UFuncTeamWork();
}
#endif