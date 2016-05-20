#include "control.h"

#include <iostream>

using namespace std;

#define maxVEL 0.27 // maximum velocity

double integralAngle = 0;
double integralVel = 0;

void Control::resetAngle()
{
    integralAngle = 0;
}

void Control::resetVel()
{
  integralVel = 0;
}


double Control::velRegulator(double targetDistanceToObject, int actualHeight, bool foundRect, FILE * visionLog)
{
  if (clrVelReg)
  {
    clrVelReg = false;
    printf("Velocity regulator has been reset\n");
    resetVel();
  }
  
  double actualDistance = 0;
  if (foundRect)
    actualDistance = 0.01/actualHeight*880; // 880 = focallength
  else
    actualDistance = 0.02/actualHeight*880; // 880 = focallength 
    

  
  double err =  actualDistance - targetDistanceToObject;
  
  double Kp = 0.8;
  
  double vel = Kp * err + masterVel;

  if (vel > maxVEL)
    vel = maxVEL;
  else if ( vel < -maxVEL)
    vel = -maxVEL;
  
  if ( actualDistance-distanceOveride > 0.30)
  {
    vel = logVel;
    fprintf(visionLog, "1\n");
  }
  else
  {
    distanceOveride = actualDistance;
    fprintf(visionLog, "0\n");
  } 
  
  logVel = vel;

  printf("Distance to object: %f\n", actualDistance);
  
  return vel;
}



int Control::angleRegulator(int midTarget, int midImg)
{
       
       

    /*
     PI regulator to calculate a reference angle to 
     the regbot.
     * takes the x-pixelpoint of the tape-line-edge as ref-input
     * and uses img.col/2 as feedback (the x-point of mid image)
     * as feedback.
     * had to invert err because of differences in regbot and
     * raspicams coordinate systems
     */


    double err = midImg - midTarget;

    double Kp = 0.016/3;

    double Ki = 0.55;


    integralAngle = integralAngle + (err * 0.03333);


    int angle = err * Kp + Ki * integralAngle + angleToGmk;
        
    logAngle = angle*M_PI/180;
    
    /*
     * 
     * This is the first method I used to control regbot.
     * Very lazy, but works surprisingly well
     * 
    if (edge < midImg-45)
    {
        if (refAngle > 80)
            angle += 4;
        else
            angle = angle + 2;
    }
    else if (edge > midImg+45) 
    {
        if (refAngle < -80)
            angle += -4;
        else
            angle = angle - 2;
    }
    else
    {
        angle = angle;
    }
    */
        
    if (angle >= 540)
        angle -= 360;
    else if (angle <= -540)
        angle += 360;
     
   
    
    return angle;

        
}