/* 
 * File:   teamwork.h
 * Author: jesper
 *
 * Created on February 22, 2016, 6:14 PM
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