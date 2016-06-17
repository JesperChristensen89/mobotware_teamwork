/**
 * @brief class to handle the TCP connections
 * 
 * @author Jesper H. Christensen, 2016
 * jesper@haahrchristensen.dk
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