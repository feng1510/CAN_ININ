#ifndef EPS_H
#define EPS_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>
#include "define.h"

using namespace std;
using namespace cv;





class EPS
{
public:
    EPS();

   /************************   EPS_1 : 0x366   *****************************/


    bool EPS_StrngWhlTorqVD;       //0=Not valid; 1=valid 

    bool EPS_WarnLamp;             //0=Fail not present; 1=Fail present

    //EPS_SteringAngle
    bool EPS_SteeringAngle_invalid_flag;
    double EPS_SteeringAngle;
    double EPS_SteeringAngle_resolution;
    double EPS_SteeringAngle_offset;
    double EPS_SteeringAngle_min;
    double EPS_SteeringAngle_max;
    int EPS_SteeringAngle_invalid_value;

    //EPS_SteeringAngleSpt
    double EPS_SteeringAngleSpt;
    double EPS_SteeringAngleSpt_resoultion;
    double EPS_SteeringAngleSpt_offset;
    double EPS_SteeringAngleSpt_min;
    double EPS_SteeringAngleSpt_max;
    int EPS_SteeringAngleSpt_invalid_value;
    
    //EPS_StrngWhlTorq
    bool EPS_StrngWhlTorq_invalid_flag;
    double EPS_StrngWhlTorq;
    double EPS_StrngWhlTorq_resolution;
    double EPS_StrngWhlTorq_offset;
    double EPS_StrngWhlTorq_min;
    double EPS_StrngWhlTorq_max;
    int EPS_StrngWhlTorq_invalid_value;


    void EPS_1_decode(struct can_frame &EPS_frame);

  /*************************   EPS_11 : 0x365 ************************************/
    int EPS_ControlMode;
    bool EPS_FailureSt;
    void EPS_11_decode(struct can_frame &EPS_11_frame);

    /************************* EPB_1 0x397  *************************************************/
    __u8 EPB_SysSt;    //EPB system status 单独的电子手刹，也放在EPS一起

    void EPB_decode(struct can_frame frame);

    void EPS_display(cv::Mat Display);

private:
    /* data */
};

























#endif