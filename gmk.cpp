#include "gmk.h"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <algorithm> 
#include <iostream>

using namespace std;

vector<int> IDs; // global vector to hold IDs

/**
 * @brief used for clearing guidemark flags
 * 
 */
GMK::GMK()
{
  gmkIsReady = false;
  gmkRunning = false;
}


/**
 * @brief adds an ID to the vector
 * 
 * @param id ID number
 * @return void
 */
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

/**
 * @brief Returns the ID with a specific index
 * 
 * @param idx index for ID
 * @return int
 */
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
  
  char IDstr[10];
  for (uint i = 0; i < IDs.size(); i++)
  {
    snprintf(IDstr, 10, "%d", IDs[i]);
    
    int temp = atoi(IDstr);
    
    ID2find = temp;
    
    if (temp/10 == ID)
    {
      printf("\nCorrect ID is found\n");
      return true;
    }
    
  }
  return false;

}

/**
 * @brief sets the running flag for guidemark
 * 
 * @param val flag
 * @return void
 */
void GMK::setRunning(bool val)
{

  gmkRunning = val;
  
}

/**
 * @brief returns running flag for guidemark
 * 
 * @return bool
 */
bool GMK::isRunning()
{
  return gmkRunning;
}




/**
 * @brief gets the size of guidemark vector
 * 
 * @return int
 */
int GMK::getGmkCount()
{
  return IDs.size();
}

/**
 * @brief clears the ID vector
 * 
 * @return bool
 */
bool GMK::clearIDs()
{
  
  IDs.erase(IDs.begin(),IDs.end());
  
  return IDs.empty();
  
}

/**
 * @brief sets the gmk ready flag
 * 
 * @param val flag
 * @return void
 */
void GMK::setReadyFlag(bool val)
{
  gmkIsReady = val;
}

/**
 * @brief gets the gmk ready flag
 * 
 * @return bool
 */
bool GMK::getReadyFlag()
{
  return gmkIsReady;
}

/**
 * @brief print ID in vector
 * 
 * @return void
 */
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

