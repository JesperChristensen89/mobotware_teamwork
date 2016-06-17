/**
 * @brief class to handle image processing
 * 
 * 
 * @author Jesper H. Christensen, 2016
 * jesper@haahrchristensen.dk
 */

#ifndef TEAMWORK_H
#define TEAMWORK_H

#include <opencv2/core/core.hpp>
#include "uart.h"

class TeamWork{
public:

    bool doWork(cv::Mat img, UART uart);
    
private:
  cv::Mat correctColors(cv::Mat);
  static bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );
  int count;

};

#endif /* LINEFOLLOWER_H */