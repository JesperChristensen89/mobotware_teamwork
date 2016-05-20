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

#define PI 3.14159265

using namespace cv;
using namespace std;

Control control;

bool first = true;
bool debug = false;
FILE * visionLog;

Mat TeamWork::correctColors(Mat img)
{

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

bool TeamWork::compareContourAreas(vector< Point > contour1, vector< Point > contour2)
{
  double i = fabs( contourArea(Mat(contour1)) );
  double j = fabs( contourArea(Mat(contour2)) );
  return ( i > j );
}


bool TeamWork::doWork(Mat img, UART uart)
{
  if (stopVisionLog)
  {
    fclose(visionLog);
  }
  
  if (first)
  {
    count = 0;
    first = false;
    printf("Starting\n");
    
    visionLog = fopen("../visionlog.txt", "w");
    
    fprintf(visionLog, "N Succes Square False\n");
  }
  

  count +=1;
  
  fprintf(visionLog, "%d ", count);
  
  if (count > 800)
  {
    char stopStr[50];
    Commands::angle(stopStr, 998);
    uart.send(stopStr);
    return false;
  }
  
  Mat imgRGB = correctColors(img.clone());
  
  
  //if (debug)  
  { 
    char saveRawStr[50];
    snprintf(saveRawStr, 50, "/home/pi/runs/raw%d.png",count);
    imwrite(saveRawStr, imgRGB);
    printf("Saved!\n"); 
  }
  
  int srcHeight = imgRGB.rows;
  int srcWidth = imgRGB.cols;
  
  Mat resized;
  resize(imgRGB,resized,Size(srcWidth/4,srcHeight/4));
  
  Mat imgHSV;
  cvtColor(resized,imgHSV, CV_BGR2HSV);
 
  
  Mat thresh;
  inRange(imgHSV, Scalar(0,120,90), Scalar(255,255,255), thresh);
  //inRange(imgHSV, Scalar(0,189,28), Scalar(11,255,255), thresh);
  
  Mat open, closed;
  Mat kernelOpen = getStructuringElement(MORPH_RECT, Size(thresh.rows/7,1));
  Mat kernelClose= getStructuringElement(MORPH_RECT, Size(thresh.rows/5,1));
  morphologyEx(thresh, open, MORPH_OPEN, kernelOpen); 
  morphologyEx(open,closed,MORPH_CLOSE, kernelClose);
  
  //kernelOpen = getStructuringElement(MORPH_RECT, Size(thresh.rows/20*3,1));
  //morphologyEx(closed,open,MORPH_OPEN, kernelOpen);
  
  vector< vector<Point> > cnts; 
  vector<Vec4i> hierarchy;
  
  findContours( closed.clone(), cnts, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE ); 
  //sort(cnts.begin(), cnts.end(), compareContourAreas);
  
  if (cnts.size() == 0)
  {
    fprintf(visionLog, "0 0 0\n");
    
    return false;
  }
  else
    fprintf(visionLog, "1 ");
  
  double m = 0;
  int mIdx = 0;
  
  for (uint i = 0; i < cnts.size(); i++)
  {
    Rect rec = boundingRect(cnts[i]);
      
    if (rec.width > 50)
    {
      Scalar temp = mean(imgHSV(rec));
      if (temp[2] > m)
      {
	m = temp[2];
	mIdx = i;
      }
    }
  } 
  
  Rect boundRed = boundingRect(cnts[mIdx]);
      
  Mat redCropped = imgRGB(Rect(boundRed.x*4,boundRed.y*4,boundRed.width*4,boundRed.height*4));
  
  Mat redHSV;
  cvtColor(redCropped,redHSV,CV_BGR2HSV);
  
  Mat redThresh;
  inRange(redHSV, Scalar(0,120,90), Scalar(255,255,255), redThresh);
  //inRange(redHSV, Scalar(0,189,28), Scalar(11,255,255), redThresh);
    
  Mat redOpen;
  
  printf("Before open\n");
  
  try
  {
    
    Mat kernelRedOpen = getStructuringElement(MORPH_RECT, Size(redThresh.rows/5,redThresh.rows/5));
    Mat inverted;
    
    bitwise_not(redThresh.clone(),inverted);
    morphologyEx(inverted, redOpen, MORPH_OPEN, kernelRedOpen);
    bitwise_not(redOpen, redOpen);
    
  }
  catch (...)
  {
    printf("Could not 'open' redThresh\n");
    redOpen = redThresh;
  }
    
  printf("After open\n");
  
  vector< vector<Point> > innerCnts; 
  vector<Vec4i> innerHierarchy;
  
  findContours( redOpen.clone(), innerCnts, innerHierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE ); 
  sort(innerCnts.begin(), innerCnts.end(), compareContourAreas);
  
  /*
  if (innerCnts.size() <= 1)
    return false;
  */
  
  
  bool squareFound = false;
  int rectHeight = 0;
  vector<Point> approx;
  int cSqX = 0, cSqY = 0;

  
  for (uint i = 1; i < innerCnts.size(); i++)
  {
    double peri = arcLength(innerCnts[i],true);
    
    approxPolyDP(innerCnts[i], approx, 0.04*peri, true);
       
    
    if (approx.size() == 4)
    {
      //Rect rect = boundingRect(innerCnts[i]);
      //rectHeight = rect.height;
      
      int a = 999;
      int b = 0;
      
      for (uint j=0; j< approx.size(); j++)
      {
	if (approx[j].y > b)
	  b = approx[j].y;
	if (approx[j].y < a)
	  a = approx[j].y;
      }
      
      rectHeight = b-a;
      
      if (rectHeight > 8 and rectHeight < 100)
      {
	Moments sqMoments = moments(innerCnts[i]);
	cSqX = (int)(sqMoments.m10 / sqMoments.m00) + boundRed.x*4;
	cSqY = (int)(sqMoments.m01 / sqMoments.m00) + boundRed.y*4;
	
	//drawContours(imgRGB,innerCnts,i,Scalar(0,255,0),1,8,innerHierarchy,10, Point(boundRed.x*4,boundRed.y*4));
	
	circle(imgRGB,Point(cSqX,cSqY), 3, Scalar(0,0,255),-1, 8, 0);
	rectangle(imgRGB, boundingRect(innerCnts[i]), Scalar(0,0,255),1,8,0);
	squareFound = true;
	
	
	break;
      }

    }
  }
   

  int angle = 0;
  double vel = 0;

  
  if (squareFound)
  {
    angle = control.angleRegulator(cSqX,imgRGB.cols/2);
    
    printf("Tracking rect\n");
    
    fprintf(visionLog, "1 ");
    
    vel = control.velRegulator(0.4, rectHeight, true, visionLog); // set ref in m from target
    
    /*
    char saveSqStr[50];
    snprintf(saveSqStr, 50, "/home/pi/runs/square%d.png",count);
    imwrite(saveSqStr, imgRGB);
    */
    
  }
  else
  { 
    fprintf(visionLog, "0 "); 
    
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
 
  
  char VAstr[20];
  snprintf(VAstr,20,"VA%.3f%.3f\n",vel,angle * PI / 180);
  uart.send(VAstr);
  
  
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

