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

#include "../aupoly/urespoly.h"
#include <urob4/uresposehist.h>
//#include <urob4/ucmdexe.h>
//#include <../libs/eigen3/Eigen/src/Eigen2Support/Geometry/Translation.h>
#include <highgui.h>
//#include <time.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <pthread.h>
#include <unistd.h>
#include <cmath>
 
TeamWork teamwork;
GMK gmk;
UART uart;

double distanceToGmk = 0;
double angleToGmk = 0;

void* uartReceiver(void* arg)
{
  while(1)
  {
    uart.receive();
    usleep(500);
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
  if (not ask4help)
  { // get all other parameters
    msg->tag.getAttValueInt("device", &camDevice);
    imgPoolIsSet = msg->tag.getAttValueInt("img", &imgPoolNum);
    msg->tag.getAttValueBool("debug", &debug, true);
    msg->tag.getAttValueBool("smrcl", &smrcl, true);
    msg->tag.getAttValueBool("setup", &setup, true);
  }
  if (setup)
  {
    getCorePointer()->postCommand(-36,"varpush struct=gmk call=\"teamwork.getGMK(0)\"\n");
    getCorePointer()->postCommand(-36,"poolpush img=13 cmd=\"teamwork img=13\"\n");
    
    uart.init();
    
    
    pthread_t uartReceiveThread;
    int ret = pthread_create(&uartReceiveThread, NULL, &uartReceiver, NULL);
    if (ret != 0)
    {
      printf("Error: pthread_create() failed\n");
      exit(EXIT_FAILURE);
    }
    
    uart.send((char*)"S=1\n");
    
    printf("\nSetup completed!\n");
    
    
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
    //uart.send((char *)"998\n");
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
      
      // there is an image, make the required analysis
      
      cv::Mat imgCV = cv::cvarrToMat(img->cvArr());
  
      if (missionStart)
      {
	uart.send((char*)"Y=1\n");
	teamwork.doWork(imgCV, uart);
      }
      
      /*
    
      switch (missionState)
      {
	case 0: // get guidemarks
	  if (not gmk.isRunning())
	  {
	    gmkImg = imgPool->getImage(55,true);
	    gmkImg->copy(img);
	    gmkImg->setName("GMK image");
	    gmkImg->updated();
	    gmk.setReadyFlag(false);
	    findGMK();
	    missionState = 1;
	  }
	  break;
	  
	case 1: // verify CRC and get distance
	  if (gmk.getReadyFlag())
	  {
	    bool IDavailable = gmk.findID(ID2find);
	    	    
	    if (IDavailable)
	    {
	      bool crcOK = verifyGMK(ID2find);
	      
	      if (crcOK)
	      {
		
		
		double gmkPose[6];
		      
		for (int i=0; i < 6; i++)
		{
		  char gmkPoseCall[50];
		  snprintf(gmkPoseCall,50,"gmk.gmk%d.gmkPose[%d]",ID2find,i);
		  
		  const char* gmkPoseCallConst = gmkPoseCall;
		  
		  getGlobalValue(gmkPoseCallConst,&gmkPose[i]);
		  

		    
		}
	
		//printf("Distance to GMK is: x=%f y=%f z=%f\nRotation is: Omega=%f Phi=%f Kappa=%f\n",gmkPose[0],gmkPose[1],gmkPose[2],gmkPose[3]*180/3.14,gmkPose[4]*180/3.14,180-gmkPose[5]*180/3.14);
		
		distanceToGmk = sqrt(pow(gmkPose[0],2) + pow(gmkPose[1],2)) - 0.1; // goto 10 cm before guidemark
		angleToGmk = M_PI/2 - acos(gmkPose[1]*0.8/(distanceToGmk+0.1));
		
		missionState = 2;
	      }
		
	      else
		missionState = 0;
	    }
	    else
	      missionState = 0;
	  }
	  break;
	  
	case 2: // ID 20 is identified and verified
	  char distanceStr[50];
	  snprintf(distanceStr,50,"D=%f\n",distanceToGmk);
	  uart.send(distanceStr);
	  
	  char angleStr[50];
	  snprintf(angleStr,50,"A=%f\n",angleToGmk);
	  uart.send(angleStr);
	  
	  uart.send((char*)"Y=1\n");
	  
	  printf("%s%s",distanceStr,angleStr);
	  
	  
	  
	  
	  missionState = 3;
	  break;
	  
	case 3:
	  break;
	  
	  
	 
      
	
      }
	
	 */
      
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
	
	const char* finalGmkIdCall = gmkIdCall;
	
	double id;
	getGlobalValue(finalGmkIdCall,&id);
	
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

void* gmkWaitThread(void *arg )
{
  
  
  usleep(1700000);
  printf("gmkWaitThread has finished\n");
  
  gmk.setRunning(false);
  
  gmk.setReadyFlag(true);
  
  
  pthread_exit(NULL);
}
  
  
  
void UFuncTeamWork::findGMK()
{
  pthread_t t1;
  int ret = pthread_create(&t1, NULL, &gmkWaitThread, NULL);
  if (ret != 0)
  {
    
    printf("Error: pthread_create() failed\n");
    exit(EXIT_FAILURE);
  }
  else
  {
    gmk.setRunning(true);
    getCorePointer()->postCommand(-36,"gmk img=55 block=0.00357");
    printf("gmkWaitThread created\n");
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