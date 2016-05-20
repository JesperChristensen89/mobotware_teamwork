/* 
 * File:   control.h
 * Author: jesper
 *
 * Created on February 25, 2016, 8:07 AM
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include "ufuncteamwork.h"
#include <stdlib.h>

class Control 
{
public:
    int angleRegulator(int, int);
    double velRegulator(double targetDistanceToObject, int actualHeight, bool foundRect, FILE * visionLog); 
    void resetAngle();
    void resetVel();
};

#endif /* CONTROL_H */