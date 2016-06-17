/**
 * @brief class to handle guidemarks
 * 
 * @author Jesper H. Christensen, 2016
 * jesper@haahrchristensen.dk
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