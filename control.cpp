#include "control.h"
#include <iostream>

using namespace std;

#define maxVEL 0.27 // maximum velocity


/**
 * @brief used for clearing speed controller
 * 
 * @return void
 */
void Control::resetAngle()
{
    integralAngle = 0;
}

/**
 * @brief speed controller
 * 
 * @param targetDistanceToObject target distance to leader
 * @param actualHeight height in pixels
 * @param foundRect flag indicating whether or not tracking point is located
 * @param visionLog used when logging
 * @return double
 */
double Control::velRegulator(double targetDistanceToObject, int actualHeight, bool foundRect, FILE * visionLog)
{
  // init variable
  double actualDistance = 0;
  
  // calc the distance to leader in meters
  if (foundRect)
    actualDistance = 0.01/actualHeight*880; // 880 = focallength
  else
    actualDistance = 0.02/actualHeight*880; // 880 = focallength 
    
  // compute speed 
  double err =  actualDistance - targetDistanceToObject;
  
  double Kp = 0.8;
  
  double vel = Kp * err + masterVel;

  // check value
  if (vel > maxVEL)
    vel = maxVEL;
  else if ( vel < -maxVEL)
    vel = -maxVEL;
  
  // skip value if change in distance is too large
  if ( actualDistance-distanceOveride < 0.30 or actualDistance < 0.4)
  {
    distanceOveride = actualDistance;
    fprintf(visionLog, "0\n");
  }
  else
  {
    
    vel = logVel;
    fprintf(visionLog, "1\n");
  } 
  
  logVel = vel;

  // print distance to leader
  printf("Distance to object: %f\n", actualDistance);
  
  return vel;
}



/**
 * @brief direction controller
 * 
 * @param midTarget x coordinate of the target point
 * @param midImg reference for controller
 * @return int
 */
int Control::angleRegulator(int midTarget, int midImg)
{
  // run the controller
  double err = midImg - midTarget;

  double Kp = 0.016/3;

  double Ki = 0.55;


  integralAngle = integralAngle + (err * 0.03333);


  int angle = err * Kp + Ki * integralAngle + angleToGmk;
  
  logAngle = angle*M_PI/180;
  
  // chech for max and min value
  if (angle >= 540)
      angle -= 360;
  else if (angle <= -540)
      angle += 360;
    
  
    
  return angle;

        
}