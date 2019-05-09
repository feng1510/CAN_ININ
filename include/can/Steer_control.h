#ifndef STEER_CONTROL_H
#define STEER_CONTROL_H


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>
#include "define.h"
#include "EPS.h"

using namespace std;
using namespace cv;

class Steer_control
{
public:
    Steer_control();
    virtual ~Steer_control();

    /******** SCU_2 0x37B *************/
    bool SCU_EpsOnReq;
    double SCU_SteerAngReq;

    struct can_frame frame;

    int16_t angle_input;
    
    void steer_control_code();
    void in_steer_auto(EPS& eps);
    void esc_steer_auto();
    //本地回环
    void steer_loop_decode(struct can_frame &frame);




    //display
    void steer_control_dis(EPS& eps, cv::Mat Display_control);




private:
    /* data */
};









#endif