#include "teamwork.h"
#include "control.h"
#include "commands.h"
#include <algorithm>
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <stdio.h>

#define PI 3.14159265 // define pi

using namespace cv;
using namespace std;

Control control; // create controller object

// globals
bool first = true;
bool debug = false; // when true images are saved to disk
FILE * visionLog;

/**
 * @brief This is used to correct the images
 * received from the picam plugin. somehow it
 * kept switching red and blue channel
 * 
 * @param img image in cv format
 * @return cv::Mat
 */
Mat TeamWork::correctColors(Mat img)
{

  // switch r and b channel
  
  Mat imgCorrect;
  vector<Mat > spl;
  split(img,spl);
  
  vector<Mat> toMerge;
  
  toMerge.push_back(spl[2]);
  toMerge.push_back(spl[1]);
  toMerge.push_back(spl[0]);
  
  merge(toMerge,imgCorrect);
  
  return imgCorrect;
  
}
/**
 * @brief used for sorting contours by area 
 * 
 * @param contour1 contour 1
 * @param contour2 contour 2
 * @return bool
 */

bool TeamWork::compareContourAreas(vector< Point > contour1, vector< Point > contour2)
{
  double i = fabs( contourArea(Mat(contour1)) );
  double j = fabs( contourArea(Mat(contour2)) );
  return ( i > j );
}


/**
 * @brief this is the main image processing method
 * 
 * @param img source image to be processed
 * @param uart uart object in order to directly communicate to follower via uart
 * @return bool
 */
bool TeamWork::doWork(Mat img, UART uart)
{
  // check if logging should stop
  if (stopVisionLog)
  {
    fclose(visionLog);
  }
  
  // used for initiating img counter + log
  if (first)
  {
    count = 0;
    first = false;
    printf("Starting\n");
    
    visionLog = fopen("../visionlog.txt", "w");
    
    fprintf(visionLog, "N Succes Square False\n");
  }
  
  count += 1;
  
  fprintf(visionLog, "%d ", count); // write contour to log
  
  // check for stop condition
  if (count > 800)
  {
    char stopStr[50];
    Commands::angle(stopStr, 998);
    uart.send(stopStr);
    return false;
  }
  
  Mat imgRGB = correctColors(img.clone());  // corrects colors on img
  
  // save src image if debugging
  if (debug)  
  { 
    char saveRawStr[50];
    snprintf(saveRawStr, 50, "/home/pi/runs/raw%d.png",count);
    imwrite(saveRawStr, imgRGB);
    printf("Saved!\n"); 
  }
  
  // get dimensions of image
  int srcHeight = imgRGB.rows;
  int srcWidth = imgRGB.cols;
  
  // resize source image
  Mat resized;
  resize(imgRGB,resized,Size(srcWidth/4,srcHeight/4));
  
  // convert to HSV
  Mat imgHSV;
  cvtColor(resized,imgHSV, CV_BGR2HSV);
 
  // threshold HSV image
  Mat thresh;
  inRange(imgHSV, Scalar(0,120,90), Scalar(255,255,255), thresh);
  //inRange(imgHSV, Scalar(0,189,28), Scalar(11,255,255), thresh);
  
  Mat open, closed; // init two Mat objects
  
  // create kernel for opening
  Mat kernelOpen = getStructuringElement(MORPH_RECT, Size(thresh.rows/7,1));
  
  // create kernel for closing
  Mat kernelClose= getStructuringElement(MORPH_RECT, Size(thresh.rows/5,1));
  
  // open thresholded image
  morphologyEx(thresh, open, MORPH_OPEN, kernelOpen); 
  
  // close opened image
  morphologyEx(open,closed,MORPH_CLOSE, kernelClose);
  
  // init vectors to hold contour points
  vector< vector<Point> > cnts; 
  vector<Vec4i> hierarchy;
  
  // find contours in the closed image
  findContours( closed.clone(), cnts, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE ); 
  //sort(cnts.begin(), cnts.end(), compareContourAreas);
  
  // if no contours found -> return
  if (cnts.size() == 0)
  {
    fprintf(visionLog, "0 0 0\n");
    
    return false;
  }
  else
    fprintf(visionLog, "1 ");
  
  // init variables used contour filtration
  double m = 0;	// will hold largest mean value-intensity
  int mIdx = 0; // will hold index to the contour with largest m
  
  // loop through contours
  for (uint i = 0; i < cnts.size(); i++)
  {
    // get bounding rect of contour
    Rect rec = boundingRect(cnts[i]);
      
    // check for realistic width
    if (rec.width > 50)
    {
      // use the rectangle to crop the resized HSV image
      // and calculate the mean for each H, S and V
      // channel within the region cropped by the rect
      Scalar temp = mean(imgHSV(rec));
      
      // find the biggest mean V-intensity
      if (temp[2] > m)
      {
	m = temp[2];
	mIdx = i;
      }
    }
  } 
  
  // get the bounding rect of the contour holding the largest average V-intensity
  Rect boundRed = boundingRect(cnts[mIdx]);
      
  // crop the full sized image using the above rect as frame
  Mat redCropped = imgRGB(Rect(boundRed.x*4,boundRed.y*4,boundRed.width*4,boundRed.height*4));
  
  // convert cropped image to HSV
  Mat redHSV;
  cvtColor(redCropped,redHSV,CV_BGR2HSV);
  
  // threshold the cropped HSV img
  Mat redThresh;
  inRange(redHSV, Scalar(0,120,90), Scalar(255,255,255), redThresh);
  //inRange(redHSV, Scalar(0,189,28), Scalar(11,255,255), redThresh);
    
  Mat redOpen; // init object to hold opened img
     
  // some try and catch was needed here
  // didn't manage to use the time to debug this and find out why this sometimes would crash the app
  // was not necessary in Python
  try
  {
    // get kernel for opening
    Mat kernelRedOpen = getStructuringElement(MORPH_RECT, Size(redThresh.rows/5,redThresh.rows/5));
    
    Mat inverted; // init image object hold inverted image
    
    // invert the thresholded image to make the square a foreground object
    bitwise_not(redThresh.clone(),inverted);
    
    // open the image
    morphologyEx(inverted, redOpen, MORPH_OPEN, kernelRedOpen);
    
    // re-invert the image
    bitwise_not(redOpen, redOpen);
    
  }
  // catch the error and do nothing about it
  catch (...)
  {
    printf("Could not 'open' redThresh\n");
    redOpen = redThresh;
  }
  
  // init vectors to hold contours
  vector< vector<Point> > innerCnts; 
  vector<Vec4i> innerHierarchy;
  
  // find contours in within the red band
  findContours( redOpen.clone(), innerCnts, innerHierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE ); 
  
  // sort the contours by area
  sort(innerCnts.begin(), innerCnts.end(), compareContourAreas);
  
  /*
  if (innerCnts.size() <= 1)
    return false;
  */
  
  bool squareFound = false; 	// flag
  int rectHeight = 0, rectWidth = 0; // holds shape of object in pixels
  vector<Point> approx;		// will hold the points to the approximated contour
  int cSqX = 0, cSqY = 0;	// coordinates of square mass centre

  // loop through found contours
  for (uint i = 1; i < innerCnts.size(); i++)
  {
    // get the perimeter in order to compute epsilon for the Douglas-Peucker algorithm
    double peri = arcLength(innerCnts[i],true);
    
    // Douglas-Peucker
    approxPolyDP(innerCnts[i], approx, 0.04*peri, true);
       
    // check if the contour is approximated by a rectangle
    if (approx.size() == 4)
    {
      // get the bounding rect and dimension of it
      Rect rect = boundingRect(innerCnts[i]);
      rectHeight = rect.height;
      rectWidth = rect.width;
          
      // for heigh-width ratio (square)
      if (rectHeight/rectWidth < 1.2 and rectHeight/rectWidth > 0.8)
      {
	// get the image moments of the square
	Moments sqMoments = moments(innerCnts[i]);
	
	// get coordinates of mass centre
	cSqX = (int)(sqMoments.m10 / sqMoments.m00) + boundRed.x*4;
	cSqY = (int)(sqMoments.m01 / sqMoments.m00) + boundRed.y*4;
	
	//drawContours(imgRGB,innerCnts,i,Scalar(0,255,0),1,8,innerHierarchy,10, Point(boundRed.x*4,boundRed.y*4));
	
	// drawing functions
	circle(imgRGB,Point(cSqX,cSqY), 3, Scalar(0,0,255),-1, 8, 0);
	rectangle(imgRGB, boundingRect(innerCnts[i]), Scalar(0,0,255),1,8,0);
	
	squareFound = true; // set found flag
	
	
	break;
      }

    }
  }
   

  int angle = 0; // init angle to folloer
  double vel = 0;// init speed to follower

  // use square data if found
  if (squareFound)
  {
    // get angle reference
    angle = control.angleRegulator(cSqX,imgRGB.cols/2);
    
    printf("Tracking rect\n");
    
    fprintf(visionLog, "1 ");
    
    // get speed ref
    vel = control.velRegulator(0.4, rectHeight, true, visionLog); // set ref in m from target
    
    /*
    char saveSqStr[50];
    snprintf(saveSqStr, 50, "/home/pi/runs/square%d.png",count);
    imwrite(saveSqStr, imgRGB);
    */
    
  }
  else // use red band data
  { 
    fprintf(visionLog, "0 "); 
    
    // get mass centre and evaluate to get angle and vel
    Moments mu = moments(innerCnts[0]);
    int cX = (int)(mu.m10 / mu.m00) + boundRed.x * 4;
    int cY = (int)(mu.m01 / mu.m00) + boundRed.y * 4;
    
    circle(imgRGB,Point(cX,cY), 3, Scalar(0,255,0),-1, 8, 0);
    
    angle = control.angleRegulator(cX, imgRGB.cols/2);
    
    vel = control.velRegulator(0.4, boundRed.height*4, false, visionLog); // set ref in m from target
  }
  
  
  
  printf("Vel: %f\n",vel);
  
  /*
  char velStr[10];
  //snprintf(velStr,10,"V=%f\n",vel);
  Commands::vel(velStr, vel);
  uart.send(velStr);
  
 
  char angleStr[10];
  //snprintf(angleStr,10,"A=%f\n",angle * PI / 180);
  Commands::angle(angleStr, angle * PI / 180);
  uart.send(angleStr);
  */
 
  // send speed and angle to follower
  char VAstr[20];
  snprintf(VAstr,20,"VA%.3f%.3f\n",vel,angle * PI / 180);
  uart.send(VAstr);
  
  // save resulting image if debugging
  if (debug)
  { 
    char saveResStr[50];
    snprintf(saveResStr, 50, "/home/pi/runs/res%d.png",count);
    imwrite(saveResStr, imgRGB);
    printf("Saved!\n");
  }
  
  
  printf("Image: %d\nAngle: %d\n", count-1,angle);
  
  
  printf("_______________________\n");

  
  return true;
}

