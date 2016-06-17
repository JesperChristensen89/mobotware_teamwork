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
 
TeamWork teamwork;  // create teamwork object
GMK gmk; // create gmk object
UART uart; // uart object
TCP tcpClient; // tcp object
FILE * logFile;
bool startLogging = false;

// will hold the distance to the guidemark
double distanceToGmk = 0;
// will hold the angle to gmk
double angleToGmk = 0;
// the speed of the follower robot
double vel = 0;
// holds the integral value for angle controller
double integralAngle = 0.0;
// radius for turning
double turnRadius = 0.0;
// will be true when the raspberry is connected to tcp server on leader
bool wifiIsConnected = false;
// flag that interrupts main thread
bool interupt = false;
// true when turn is with radee
bool radeeISR;
// hold turn radius from leader
double interuptRadee;
// used for overriding the current angle when the leader is turning
double angleOveride = 0;
// used for holding the distance up to leader, for when turning
double distanceOveride = 0.40;
// state control of the "interrupt"-routines
int overideState = 0;
// defines the speed for log
double logVel = 0;
// defines angle for log
double logAngle = 0;
// defines leaders speed
double masterVel = 0;
// count the application loop
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

/**
 * @brief method to check for incoming uart data. this method runs in a separate thread
 * 
 * @param arg nothing
 * @return void*
 */

void* uartReceiver(void* arg)
{
  while(1)
  {
    uart.receive();
    usleep(500);
  }
  
  pthread_exit(NULL);
}

/**
 * @brief method to check for incoming tcp data. this method runs in a separate thread
 * 
 * @param arg nothing
 * @return void*
 */

void* tcpReceiver(void* arg)
{
  while(1)
  {
    // only if connected to leader
    if (wifiIsConnected)
    {
      tcpClient.receive();
      usleep(500);
    }
  }
  
  pthread_exit(NULL);
}

/**
 * @brief destructor
 * 
 */
UFuncTeamWork::~UFuncTeamWork(){}

  

/**
* @brief The main function called by mobotware when pushing images from picam to the plugin
* 
* @param msg msg from mobotware terminal	
* @param extra ...
* @return bool
*/
  
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
    msg->tag.getAttValueBool("debug", &debug, true); // not really used
    msg->tag.getAttValueBool("smrcl", &smrcl, true); // not used
    msg->tag.getAttValueBool("setup", &setup, true); // sets up and starts the system
    msg->tag.getAttValueBool("ready", &regbotreadytogo, true); 
    msg->tag.getAttValueInt("M",&missionState); // changes mission on regbot
  }
  if (setup) // this routine will setup mobotware and start the system
  {
    // push the gmk struct data to the getGMK method
    getCorePointer()->postCommand(-36,"varpush struct=gmk call=\"teamwork.getGMK(0)\"\n");
    // push images from picam to this plugin
    getCorePointer()->postCommand(-36,"poolpush img=13 i=1 cmd=\"teamwork img=13\"\n");
    
    // init the uart
    uart.init();
    
    // setup the uart receiver thread
    pthread_t uartReceiveThread;
    int ret = pthread_create(&uartReceiveThread, NULL, &uartReceiver, NULL);
    if (ret != 0)
    {
      printf("Error: pthread_create() failed\n");
      exit(EXIT_FAILURE);
    }
    
    // setup the tcp receiver thread
    pthread_t tcpReceiveThread;
    ret = pthread_create(&tcpReceiveThread, NULL, &tcpReceiver, NULL);
    if (ret != 0)
    {
      printf("Error: pthread_create() failed\n");
      exit(EXIT_FAILURE);
    }
    
    // send start command to follower
    uart.send((char*)"S=1\n");
    
    // create logfile
    logFile = fopen("../raspilog.txt", "w");
            
    printf("\nSetup completed!\n");
    
    
    result = true;
  }
  // send "ready to go" to leader when follower is ready
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
  // stops the follower robot
  else if (stopRegbot)
  {
    // set the mission on follower to stop
    uart.send((char*)"M=3\n");
    uart.send((char *)"A=998\n");
    // shut down log
    startLogging = false;
    fclose(logFile);
    
    printf("Sending stop command\n");
    
    stopVisionLog = true;
  }
  else  // this is where the fun starts
  { 
    // grab the image from the pool      
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
    
    if (result) // image is ready - let's follow that leader robot!
    { 
      
      // make sure client is connected to server
      if (not wifiIsConnected)
      {
	// this should not be hard coded, but as the IP did not change, like ever, I didn't use any resources to fix it
	wifiIsConnected = tcpClient.conn("10.0.1.161",1336);
	if (not wifiIsConnected)
	  return false;
      }
         
      // increase the loop counter
      loopcounter += 1;
         
      // this is the "interrupt"-routine. This flag will be set if the leader has sent any information that calls for execution by the main thread
      if (interupt)
      {
	switch (overideState) // state controlled
	{
	  case 0:
	    
	    // check if turn is 180 deg or with radee
	    if (angleOveride != 3.14 || not radeeISR ) 
	    {
	      // prepare new angle for leader
	      printf("Sending overide angle\n");
	    
	      char overrideAngleStr[20];
	      snprintf(overrideAngleStr,20, "OA=%f\n", angleOveride);
	      uart.send(overrideAngleStr);
	      
	      angleToGmk += (angleOveride*180/M_PI); // remember to add new angle to controller
	    }
	    
	    if (radeeISR) // prepare turn radius to follower
	    {
	      printf("Sending overide radee\n");
	    
	      char overrideRadeeStr[20];
	      snprintf(overrideRadeeStr,20, "T=%f\n", interuptRadee+0.05);
	      uart.send(overrideRadeeStr);
 	      
	    }
	    
	    overideState = 1;
	    break;
	    
	  case 1:
	    if(regbotReady) // if the follower is ready, continue
	    {
	      regbotReady = false;
	      
	      printf("Sending overide distance\n");
	    
	      // prepare the "command-based" driving distance for follower
	      char overrideDistStr[20];
	      snprintf(overrideDistStr,20, "OD=%f\n", distanceOveride+0.13); // +13cm to get the distance to leaders turning point
	      uart.send(overrideDistStr);
	      
	      overideState = 2;
	    }
	    
	    break;
	    
	    
	  case 2: // this decides which event should be executed on the follower
	    if(regbotReady)
	    {
	      regbotReady = false;
	      printf("Sending overide mission\n");
	      
	      if (angleOveride == 3.14) // if turn is 180 deg, make follower go around
	      {

		uart.send((char*)"M=10\n");
		//startLogging = true;
		angleToGmk = 0.0;
		integralAngle = 0;
	      }
	      else if (radeeISR) // if turn is with radee, make follower turn with radee
	      {
		uart.send((char*)"M=60\n");
		angleToGmk += 3*180;

		if (angleToGmk >= 540)
		  angleToGmk -= 360;
		else if (angleToGmk <= -540)
		    angleToGmk += 360;
	      }
	      else // perform a normal turn
		uart.send((char*)"M=8\n");
	      
	      overideState = 21;
	    }
	    break;

	    case 21: // 2.1 -> subroutine of 2
	    if (radeeISR) // this is created to cope with some strange bugs i got from the follower robot
	    {
	      if (regbotReady)
	      {
		regbotReady = false;
		radeeISR = false;
		overideState = 3;
	      }
	    }
	    else
	    	overideState = 3;
	    break;
	    
	  case 3: // send confirmation of completed routine to leader
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
	
	return true; // return here, as long the main thread must stay interrupted
      }
	
      // this is used for letting the leader start the log on both itself and the follower at ~the same time
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
		
            
     
      // with the "interrupts" handled, get on with it!
      
      cv::Mat imgCV = cv::cvarrToMat(img->cvArr());  // convert received img to cv format

      // this is state machine controlling each phase of the system
      // e.g. it must first start with connecting to leader
      // then searching for leader
      // then driving to leader
      // then following leader
      switch (missionState)
      {
	case 0: // only move on if wifi is connected
	  if (wifiIsConnected)
	  {
	    tcpClient.sendData("start\n"); // set a ready flag on leader
	    printf("Starting guidemark search\n");
	    missionState = 1;
	  }
	  break;
	
	case 1: // only get gmk if it is not currently running 
	  if (not gmk.isRunning() and tcpClient.regbotStarted)
	  {
	    //tcpClient.sendData("readyCmd\n");
	    uart.send((char*)"Y=1\n"); // set a ready flag on follower
	    // copy received image to new pool, in order not to lock it
	    gmkImg = imgPool->getImage(55,true);
	    gmkImg->copy(img);
	    gmkImg->setName("GMK image");
	    gmkImg->updated();
	    gmk.setReadyFlag(false);
	    
	    // send the GMK image to evaluating in the guidemark plugin
	    findGMK();
	    
	    // increase the tries on this image
	    gmkTries += 1;
	    
	    missionState = 2;
	  }
	  break;
	  
	case 2: // verify CRC and get location data
	  if (gmk.getReadyFlag())
	  {
	    // seach through the IDs to check if the ID to locate is available
	    bool IDavailable = gmk.findID(ID2find);
	    	    
	    if (IDavailable) // process if it is available
	    {
	      bool crcOK = verifyGMK(ID2find); // verify the ID by checking the CRC code
	      
	      if (crcOK) // if ok -> proceed
	      {
		
		// check for left or right side
		char IDstr[10];
		snprintf(IDstr, 10, "%d", ID2find);
	
		if ( IDstr[strlen(IDstr)-1] == '0')
		{
		  printf("Left side!\n");
		  leftSide = true;
		}
		else           
		{
		  leftSide = false;
		  printf("Right side!\n");
		}
		
		// extract the guidemark's location data
		for (int i=0; i < 6; i++)
		{
		  char gmkPoseCall[50];
		  snprintf(gmkPoseCall,50,"gmk.gmk%d.gmkPose[%d]",ID2find,i);
		  
		  const char* gmkPoseCallConst = gmkPoseCall;
		  
		  getGlobalValue(gmkPoseCallConst,&gmkPose[i]);
		
		}
		
		// print the data
		printf("Distance to GMK is: x=%f y=%f z=%f\nRotation is: Omega=%f Phi=%f Kappa=%f\n",gmkPose[0],gmkPose[1],gmkPose[2],gmkPose[3]*180/3.14,gmkPose[4]*180/3.14,gmkPose[5]*180/3.14);
		
		missionState = 3;
			
		
	      }
		
	      else
	      {
		if (gmkTries == 2) // if the same image have been evaluated twice, move on!
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
	 
	case 3: // we have now got the correct ID, and the location data is available
	 
	  printf("Calculating route\n");
	  
	  // there is a situation for both the left side and right side
	  // this is due the regbots not being able of completing a turn with
	  // turn radius to both sides
	  if (leftSide)
	    distanceToGmk = 0.40 - (cos(gmkPose[5]+M_PI/2)*gmkPose[0] + gmkPose[1]); // y direction drive
	  else
	    distanceToGmk = 0.40 + (cos(gmkPose[5]+M_PI/2)*gmkPose[0] - gmkPose[1]);

	  // get the angle for alignment
	  angleToGmk = angleToGmk + gmkPose[5] + M_PI/2;// - M_PI/3; // align with target
	  
	  if (not leftSide)
	    angleToGmk -= M_PI;
	  
	  // calculate the turn radius
	  turnRadius = (sin(gmkPose[5]-M_PI/2) * gmkPose[0])/2;
	  
	  // make sure angle is within limits
	  if (angleToGmk >= 2*M_PI)
	    angleToGmk -= 2*M_PI;
	  else if (angleToGmk <= -2*M_PI)
	      angleToGmk += 2*M_PI;
	  
	  // start by sending the new direction reference to follower
	  char angleStr[50];
	  snprintf(angleStr,50,"A=%.3f\n",angleToGmk);
	  uart.send(angleStr);
	  
	  
	  
	  missionState = 4;
	  
	 
	  break;
	  
	case 4: 
	  if (regbotReady) // send the calculated distance to the follower
	  {
	    regbotReady = false;
	  
	    printf("Sending distance\n");
	    char distanceStrY[50];
	    snprintf(distanceStrY,50,"D=%f\n",distanceToGmk);
	    uart.send(distanceStrY);
	    
	    
	    missionState = 5;
	  }
	  break;
	  
	case 5: // make it drive the calculated distance
	  if (regbotReady)
	  {
	    regbotReady = false;
	    
	    printf("Sending mission 4\n");
	   
	   
	    uart.send((char*)"M=4\n");
	    
	    
	    missionState = 6;
	  }
	  break;
	  
	case 6: // a simple confirmation of received information
	  if (regbotReady) 
	  {
	    regbotReady = false;
	    
	    printf("Starting driving\n");
	    
	    missionState = 7;
	  }
	  break;
	  
	case 7: // position follower behind leader
	  if (regbotReady)
	  {
	    regbotReady = false;
	    
	    if (leftSide) // use turn radius for left side
	    {
	      printf("Sending turnradius\n");
	      
	      char turnStr[50];
	      snprintf(turnStr,50,"T=%f\n",turnRadius);
	      uart.send(turnStr);
	      missionState = 8;
	    }
	    else // use drive-turn-drive-turn commands for right side
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
	  
	case 71: // used only for right side
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
	  
	case 8:  // access the correct mission states on follower
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
	  
	case 91: // again only used for right side
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
	  
	case 10: // wait a bit before sending a ready command to leader
	  if (regbotReady and loopcounter > 15)
	  {
	    regbotReady = false;
	    printf("Ready to go\n");
	    uart.send((char*)"M=98\n"); // clear pose 
	    
	    tcpClient.sendData("readyToGo\n"); // send ready to leader
	    
	    loopcounter = 0;
	    
	    missionState = 11;
	    
	  }
	  break;
	 
	case 11: // when leader has confirmed the ready command, it will proceed
	  if (regbotReady and leaderProceeding)
	  {
	    leaderProceeding = false;
	    regbotReady = false;
	    angleToGmk = 0.0;
	    missionState = 12;
	    
	  }
	  else if (loopcounter > 15) // send again if no response
	  {
	    missionState = 10;
	    regbotReady = true;
	    wifiIsConnected = false;
	    printf("No answer from Leader - trying again!\n");
	  }
	    
	    
	  break;
	  
	case 12: // clear to angle reference and move on
	  {
	    missionState = 13;
	    angleToGmk = 0.0;
	    
	    startLogging = true;
	  }
	  break;
	  
	case 13: // this will run the image processing and continue to do so until stopped. 
	  teamwork.doWork(imgCV, uart);
	  if (startLogging)
	    fprintf(logFile, "%f %f %f\n", distanceOveride, logVel, logAngle);
	  break;
	  
	default:
	  break;

      }    
    }
    else // error message to mobotware
    {
      snprintf(reply, MRL, "failed, got image %s, got camera %d %s\n",
	      bool2str(img != NULL), camDevice, bool2str(cam != NULL));
      sendWarning(msg, reply);
    }
  }

  // return true if the function is handled with a positive result
  return result;
}
  
  
  /**
   * @brief this creates a method that the "push" in mobotware is able to call
   * 
   * @param name name of method to call
   * @param paramOrder ...
   * @param strings ...
   * @param pars ...
   * @param value ...
   * @param returnStruct ...
   * @param returnStructCnt ...
   * @return bool
   */
  bool UFuncTeamWork::methodCall(const char* name, const char* paramOrder, char** strings, const double* pars, double* value, UDataBase** returnStruct, int* returnStructCnt)
{
  
  gmk.setReadyFlag(true);
  gmk.setRunning(false);
  
  bool result = true;
  UPosition pos;

  // this will get the data from the guidemark, and add the ID to object within this plugin
  if ((strcasecmp(name, "getGMK") == 0))
  {
    double countID = 0;
    // start by getting the numbers of gmk recognized
    getGlobalValue("gmk.count",&countID);

    // clear id object
    gmk.clearIDs();
    
    if (countID != 0)
    {
      // get the number of ids. had to be cast and added 0.5!
      int IDs = (int)(countID+0.5);
      
      // loop through the IDs
      for (int i=0; i < IDs; i++)
      {
	char gmkIdCall[50];
	snprintf(gmkIdCall,50,"gmk.IDs[%d]",i);
	
	// get the code of gmk and not index
	double id;
	getGlobalValue(gmkIdCall,&id);
	
	int idINT = (int)(id+0.5);
		
	// add it to the object
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
  
/**
* @brief this will post the command to evaulate the gmk image
* 
* @return void
*/
void UFuncTeamWork::findGMK()
{
  // set the gmk running flag
  gmk.setRunning(true);
  // post the command that will evaluate gmk image
  getCorePointer()->postCommand(-36,"gmk img=55 block=0.00571");
  
  printf("gmk-command posted\n");

}

/**
 * @brief this will verify the ID by checking the CRC-code
 * 
 * @param ID id to verify
 * @return bool
 */
bool UFuncTeamWork::verifyGMK(int ID)
{
  
  double crcOK = 0;
  
  // get the CRC flag from gmk-plugin
  char getCrcOK[50];
  snprintf(getCrcOK,50,"gmk.gmk%d.crcOK",ID);
  const char* finalGetCrcOK = getCrcOK;
  getGlobalValue(finalGetCrcOK,&crcOK);
  
  printf("Crc check: %f\n",crcOK);
  
  // return correspondingly
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
 * @brief Creates the variables and methods available for other plugins and mobotware
 * 
 * @return void
 */
void UFuncTeamWork::createBaseVar()
{
    varPoolImg  = addVar("poolImg", 45.0, "d", "(r/w) first image pool number to use");
    addMethod("getGMK", "d", "Get GMK");

  }
  
#ifdef LIBRARY_OPEN_NEEDED
UFunctionBase * createFunc()
{ // called by server to create an object of this type
  /** replace 'UFuncTeamWork' with your classname, as used in the headerfile */
  return new UFuncTeamWork();
}
#endif