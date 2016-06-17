#include "tcp.h"
#include "ufuncteamwork.h"
#include <stdio.h>

using namespace std;

/**
 * @brief Class constructor
 * 
 */
TCP::TCP()
{
  sock = -1;
  port = 0;
  address = "";
  regbotStarted = false;
  clrVelReg = false;
}


/**
 * @brief used for connecting to a tcp server
 * 
 * @param address ip address
 * @param port listening port
 * @return bool
 */
bool TCP::conn(string address, int port)
{
  // create socket
  if (sock == -1)
  {
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1)
    {
      perror("Could not create socket");
    }
    printf("Socket created\n");
  }
  else
  {
    // ok
  }
  
  //setup address structure
  if(inet_addr(address.c_str()) == -1)
  {
      struct hostent *he;
      struct in_addr **addr_list;
	
      //resolve the hostname, its not an ip address
      if ( (he = gethostbyname( address.c_str() ) ) == NULL)
      {
	  //gethostbyname failed
	  herror("gethostbyname");
	  printf("Failed to resolve hostname\n");
	    
	  return false;
      }
	
      //Cast the h_addr_list to in_addr , since h_addr_list also has the ip address in long format only
      addr_list = (struct in_addr **) he->h_addr_list;

      for(int i = 0; addr_list[i] != NULL; i++)
      {
	  
	  server.sin_addr = *addr_list[i];
	    
	    
	  break;
      }
  }
    
  //plain ip address
  else
  {
      server.sin_addr.s_addr = inet_addr( address.c_str() );
  }
    
  server.sin_family = AF_INET;
  server.sin_port = htons( port );
    
  //Connect to remote server
  if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
  {
      printf("connect failed. Error\n");
      return false;
  }
    
  printf("Connected\n");
  return true;
}

/**
 * @brief used for receiving incoming data from TCP
 * 
 * @return void
 */
void TCP::receive()
{
    
    char buffer[512] = "";
    
    
    //Receive a reply from the server
    if( recv(sock , buffer , sizeof(buffer) , 0) < 0)
    {
        puts("recv failed");
    }
    // wait until current interupt is done
    while (interupt == true);
    // check for leader started
    if (strncmp(buffer, "started", 7) == 0)
    {
      regbotStarted = true;
      printf("Regbot started\n");
    }
    // check for turn
    else if (strncmp(buffer, "A=",2) ==0)
    {
           
      // get the new direction
      char angleStr[10] = "";
      for (uint i = 0; i < strlen(buffer)-4; i++)
      {
	angleStr[i] = buffer[i+2];
      }
                  
      angleOveride = atof(angleStr);
      
      // set flag for processing by main thread
      interupt = true;
      
      // show received turn on screen
      printf("******************* INTERUPT *********************\n");
      printf("Turn received: %f\n", angleOveride);
      
      overideState = 0;
      
      // send confirmation to leader
      sendData("readyToGo\n");
      
    }
    // check for turn with radius
    else if (strncmp(buffer, "R=",2) ==0)
    {
      
      //printf("buffer contains: %s\n", buffer);
      
      char radeeStr[10] = "";
      for (uint i = 0; i < strlen(buffer)-4; i++)
      {
	radeeStr[i] = buffer[i+2];
      }
                
      // set flag indicating a turn with radee og get radee
      interuptRadee = atof(radeeStr);
      radeeISR = true;
      interupt = true;
      
      printf("******************* INTERUPT *********************\n");
      printf("Radee received: %f\n", interuptRadee);
      
      overideState = 0;
      
      // send confirmation to leader
      sendData("readyToGo\n");
      
    }
    // check for change in speed
    else if (strncmp(buffer, "V=",2) == 0)
    {
      
      // get the new speed
      char velStr[10] = "";
      for (uint i = 0; i < strlen(buffer)-4; i++)
      {
	velStr[i] = buffer[i+2];
      }
      masterVel = atof(velStr);
      
      // show received speed on screen
      printf("******************* VELOCITY COMMAND *********************\n");
      printf("Vel received: %f\n", masterVel);
      
      
      // send confirmation to leader
      sendData("readyToGo\n");
      
      
    }
    // check for start log command
    else if (strncmp(buffer, "startLog", 8) == 0)
    {
      printf("******************* LOG COMMAND *********************\n");
     
      // start logging from raspberry
      startTeensyLog = true;
      
      // send confirmation
      sendData("readyToGo\n");
    }
    
    // check for proceeding command
    else if (strncmp(buffer,"proceeding", 10) == 0)
    {
	
      // set flag
      leaderProceeding = true;
      
      printf("******************* LEADER IS PROCEEDING *********************\n");
      
      // send confirmation to leader
      sendData("readyToGo\n");
    }
    

}
 
 /**
  * @brief used for sending data to leader robot
  * 
  * @param data string containing the data
  * @return bool
  */
 bool TCP::sendData(string data)
{
  //Send some data
  if( send(sock , data.c_str() , strlen( data.c_str() ) , 0) < 0)
  {
      perror("Send failed : ");
      return false;
  }
  
    
  return true;
}
