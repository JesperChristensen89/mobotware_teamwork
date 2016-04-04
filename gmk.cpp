#include "gmk.h"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <algorithm> 
#include <iostream>

using namespace std;

vector<int> IDs;

GMK::GMK()
{
  gmkIsReady = false;
  gmkRunning = false;
}


void GMK::addID(int id)
{
  vector<int>::iterator it;
  it = find (IDs.begin(), IDs.end(), id);
  
  if (it == IDs.end())
  {
    // ID not there - put it in
    IDs.push_back(id);
  }
  


}

int GMK::getIDs(int idx)
{
  return IDs[idx];
}

/**
 * @brief returns the index of the wanted ID
 * 
 * @param ID is the id you want to search for
 * @return int
 */
bool GMK::findID(int ID)
{
  vector<int>::iterator it;
  it = find (IDs.begin(), IDs.end(), ID);
  
  if (it != IDs.end())
    return true;
  else
    return false;
  

}

void GMK::setRunning(bool val)
{

  gmkRunning = val;
  
}

bool GMK::isRunning()
{
  return gmkRunning;
}




int GMK::getGmkCount()
{
  return IDs.size();
}

bool GMK::clearIDs()
{
  
  IDs.erase(IDs.begin(),IDs.end());
  
  return IDs.empty();
  
}

void GMK::setReadyFlag(bool val)
{
  gmkIsReady = val;
}

bool GMK::getReadyFlag()
{
  return gmkIsReady;
}

void GMK::printGmkId()
{
  
  if (IDs.empty())
    cout << "No GMKs was found!" << endl;
  else
  {
    for (vector<int>::const_iterator i = IDs.begin(); i != IDs.end(); ++i)
      cout << *i << ' ' << endl;
  }
    
}

