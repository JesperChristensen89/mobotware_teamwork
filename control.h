/* 
 * File:   control.h
 * Author: jesper
 *
 * Created on February 25, 2016, 8:07 AM
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include <stdlib.h>

class Control
{
public:
    int regulator(int, int);
    void reset();
};

#endif /* CONTROL_H */