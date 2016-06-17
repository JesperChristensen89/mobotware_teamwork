/**
 * @brief class to handle the speed
 * and direction controller
 * 
 * @author Jesper H. Christensen, 2016
 * jesper@haahrchristensen.dk
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
};

#endif /* CONTROL_H */