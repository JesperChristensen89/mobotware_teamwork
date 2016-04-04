/* 
 * File:   gmk.h
 * Author: jesper
 *
 * Created on February 22, 2016, 6:14 PM
 */

#ifndef GMK_H
#define GMK_H

#include "ufuncteamwork.h"

class GMK 
{
public:

  GMK();
  
  void addID(int);
    
  int getIDs(int);
  bool findID(int);
  bool clearIDs();
  
  int getGmkCount();
  
  bool getReadyFlag();
  void setReadyFlag(bool);
  void printGmkId();
  
  void setRunning(bool);
  bool isRunning();
  
private:
  
  bool gmkIsReady;
  bool gmkRunning;
   

};

#endif /* GMK_H */