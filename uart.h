/* 
 * File:   UART.h
 * Author: jesper
 *
 * Created on February 22, 2016, 5:33 PM
 */

#ifndef UART_H
#define UART_H

extern bool missionStart;
extern bool regbotReady;

class UART
{
public:
    void init();
    void send(char*);
    void receive();

    
    
};


#endif /* UART_H */