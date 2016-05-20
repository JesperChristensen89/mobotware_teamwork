/* 
 * File:   TCP.h
 * Author: jesper
 *
 * Created on February 22, 2016, 5:33 PM
 */

#ifndef TCP_H
#define TCP_H

#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include<string.h>    //strlen
#include<string>  //string

class TCP
{
public:
  TCP();
  bool conn(std::string, int);
  bool sendData(std::string data);
  void receive();
  
  bool regbotStarted;
  
private:
  int sock;
  std::string address;
  int port;
  struct sockaddr_in server;
  
  

    
    
};


#endif /* TCP_H */