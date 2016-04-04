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
 
  
  if (first)
  {
    count = 0;
    first = false;
    printf("Starting\n");
  }

  count +=1;
  
  if (count > 300)
  {
    char stopStr[50];
    Commands::angle(stopStr, 998);
    uart.send(stopStr);
    return false;
  }
  
  Mat imgRGB = correctColors(img.clone());
 
  /*
  if (debug)
  {
    char saveRawStr[50];
    snprintf(saveRawStr, 50, "/home/pi/runs/raw%d.png",count);
    imwrite(saveRawStr, imgRGB);
  }
  */
  
  int xOffset = 10;
  int yOffset = imgRGB.rows/3*2;
  
  Rect roi = Rect(xOffset,yOffset, imgRGB.cols-2*xOffset, 80);
  Mat img_roi = imgRGB(roi);
  
  //rectangle(imgRGB,roi,Scalar(255,255,255),1,8,0);
  
  /*
  if (debug)
  {
    char saveRoiStr[50];
    snprintf(saveRoiStr, 50, "/home/pi/runs/roi%d.png",count);
    imwrite(saveRoiStr, img_roi);
  }
  */
  
  Mat imgHSV;
  cvtColor(img_roi, imgHSV, CV_BGR2HSV);
  
  Mat blur;
  GaussianBlur(imgHSV, blur, Size(3,3), 2);
  
  Mat thresh;
  inRange(blur, Scalar(0,154,90), Scalar(255,255,255), thresh);
  
  
  int sizeForKernel = 40;
  
  Mat closed;
  Mat structure = getStructuringElement(MORPH_RECT, Size(sizeForKernel,sizeForKernel));
  morphologyEx(thresh, closed, MORPH_CLOSE, structure); 
  
  Mat redArea;
  bitwise_and(thresh, closed, redArea);
  
  
  
  
  
  /*
  
  Mat structure = getStructuringElement (MORPH_RECT, Size(1,sizeForKernel));
  Mat e, d;
  erode(thresh, e, structure, Point(-1,-1));
  dilate(e,d,structure, Point(-1,-1));
  
  structure = getStructuringElement(MORPH_RECT, Size(sizeForKernel,1));
  
  bitwise_not(d,d);
  
  erode(d,e,structure);
  dilate(e,d,structure);
  
  bitwise_not(d,d);
  
  if (debug)
  {
    char saveCloseStr[50];
    snprintf(saveCloseStr, 50, "/home/pi/runs/close%d.png",count);
    imwrite(saveCloseStr, d);
  }
  */
  vector< vector<Point> > cnts; 
  vector<Vec4i> hierarchy;
  
  findContours( redArea.clone(), cnts, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE ); 

  if (cnts.size() == 0)
  {
    uart.send((char*)"C=1\n");
    return false;
  }
  
  sort(cnts.begin(), cnts.end(), compareContourAreas);
  
  /*
  if (debug)
  {
    drawContours(img_roi,cnts,-1,Scalar(255,255,255),1,8);
    char saveCntsStr[50];
    snprintf(saveCntsStr, 50, "/home/pi/runs/cnts%d.png",count);
    imwrite(saveCntsStr, img_roi);
  }
  */
  
  Rect boundRec = boundingRect(cnts[0]);
  float outerContArea = contourArea(cnts[0]);
  boundRec.x += xOffset;
  boundRec.y += yOffset;
  rectangle(imgRGB,boundRec, Scalar(0,255,0),1,8,0);
  
  Moments mu = moments(cnts[0]);
  int cX = (int)(mu.m10 / mu.m00) + xOffset;
  int cY = (int)(mu.m01 / mu.m00) + yOffset;

  circle(imgRGB,Point(cX,cY),3, Scalar(0,255,0),-1, 8, 0);
  
  bool squareFound = false;
  double rectArea = 0;
  vector<Point> approx;
  int cSqX = 0, cSqY = 0;
  
  if (cnts.size() <= 1)
    printf("No inner contours!!\n");
  else
    printf("Inner contours found!\n");
  
  for (uint i = 1; i < cnts.size(); i++)
  {
    //printf("Getting the arcLength\n");
    double peri = arcLength(cnts[i],true);
    
    //printf("Getting approxPolyDP\n");
    approxPolyDP(cnts[i], approx, 0.04*peri, true);
    
    //printf("Points in approxPolyDP: %d\n",approx.size());
    
    if (contourArea(cnts[i]) > 150 and approx.size() == 4)
    {
      
      //printf("Found rect - getting moments\n");
      Moments sqMoments = moments(cnts[i]);
      cSqX = (int)(sqMoments.m10 / sqMoments.m00) + xOffset;
      cSqY = (int)(sqMoments.m01 / sqMoments.m00) + yOffset;
      
      //printf("Got moments and center\n");
      if (cSqX > boundRec.x+ xOffset +25 and cSqX < boundRec.x+xOffset+boundRec.width-25)
      {
	
	rectArea = contourArea(cnts[i]);
	printf("Rect area: %f\n", rectArea);
	//printf("Valid rect found - drawing cont\n");
	//drawContours(imgRGB,innerCnts,innerCnts.size()-1,Scalar(0,0,255),1,8,innerHierarchy,0,Point(xOffset,yOffset));
	//printf("Drawing circle\n");
	circle(imgRGB,Point(cSqX,cSqY), 3, Scalar(0,0,255),-1, 8, 0);
	squareFound = true;
	break;
      }
    }
  }
  
 
  printf("Outer area: %f\n",outerContArea);
  
  
  int angle = 0;
  if (squareFound)
  {
    angle = control.regulator(cSqX,imgRGB.cols/2);
    printf("Tracking rect\n");
  }
  else
  {
    angle = control.regulator(cX, imgRGB.cols/2);
  }
  
  char angleStr[50];
  Commands::angle(angleStr, angle * PI / 180);
  uart.send(angleStr);

  if (outerContArea > 30000 or rectArea > 1500)
  {
    if (outerContArea > 43000 or rectArea > 1800)
    {
      printf("Backing up\n");
      uart.send((char*)"C=2\n");
    }
      
    else
    {
      printf("Holding pos\n");
      uart.send((char*)"C=1\n");
    }
  }
  else
    uart.send((char*)"C=0\n");



  
  printf("Image: %d\nAngle: %d\n", count-1,angle);
  
  
  /*
  if (debug)
  {
    char saveStr[50];
    snprintf(saveStr, 50, "/home/pi/runs/res%d.png", count);
    imwrite(saveStr,imgRGB);
  }
  */
  
  printf("_______________________\n");
  
  
  
  return true;
}

