#include "control.h"
#include <iostream>

using namespace std;

double integral = 0;

void Control::reset()
{
    integral = 0;
}

int Control::regulator(int edge, int midImg)
{
       
        

    /*
     * this should have been a PI regulator
     * taking current angle as feedback and
     * reference angle as input
     * 
     * could not get it to work
     * 
    double err = refAngle - angle;
    double Kp = 0.1;
    double Ki = 2;
    integral = integral + (err * 0.03333);
    angle = err * Kp + Ki * integral;
    */

    /*
     PI regulator to calculate a reference angle to 
     the regbot.
     * takes the x-pixelpoint of the tape-line-edge as ref-input
     * and uses img.col/2 as feedback (the x-point of mid image)
     * as feedback.
     * had to invert err because of differences in regbot and
     * raspicams coordinate systems
     */


    double err = midImg - edge;

    double Kp = 0.016/3;

    double Ki = 0.55;


    integral = integral + (err * 0.03333);


    int angle = err * Kp + Ki * integral;
        
    
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