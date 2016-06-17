/**
 * @brief class to handle the UART serial connection
 * 
 * 
 * @author Jesper H. Christensen, 2016
 * jesper@haahrchristensen.dk
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