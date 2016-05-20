/* 
 * File:   ufuncteamwork.h
 * Author: jesper
 *
 * Created on February 22, 2016, 6:14 PM
 */

#ifndef UFUNCTEAMWORK_H
#define UFUNCTEAMWORK_H


#include <ucam4/ufunctioncambase.h>

extern int ID2find;
extern double angleToGmk;
extern bool clrVelReg;

extern bool interupt;
extern double angleOveride;
extern double distanceOveride;
extern int overideState;
extern double logAngle;
extern double logVel;
extern double masterVel;
extern bool startTeensyLog;
extern bool stopVisionLog;
extern bool leaderProceeding;

/**
Vision based collaborating mobile robots
@author Jesper H. Christensen
*/
class UFuncTeamWork : public UFunctionCamBase
{
public: 
  
    /**
  Constructor */
  UFuncTeamWork()
  {
    setCommand("teamwork", "teamwork", "vision based teamwork application (compiled " __DATE__ " " __TIME__ ")");
    // create global variables
    createBaseVar();    
    missionState = 0;
    
    gmkTries = 0;
    
  };
  
  
  
  virtual ~UFuncTeamWork();
  
  virtual bool handleCommand(UServerInMsg * msg, void * extra);
  
  bool methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt);
  
protected:
  
  void createBaseVar();
  
private:

  UVariable * varPoolImg;
  UVariable * varLeft;
  UVariable * varWhite;
  UVariable * varErrThresh;
  void findGMK();
  bool verifyGMK(int);
  
  int missionState;
  // how many times findGMK has been launched at current position
  int gmkTries;
  
  double gmkPose[6];

};
  

  
#endif /* UFUNCTEAMWORK_H */