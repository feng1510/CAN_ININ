#ifndef ELSE_H
#define ELSE_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>
#include "define.h"

using namespace std;
using namespace cv;

class ELSE
{
public:
    ELSE();
    virtual ~ELSE();

    /*************** GSM_1_id 0x190 **************/
    __u8 GSM_GearShiftLeverPstReq;

    /**************** SAS_1_id 0x264 ***************/
    bool SAS_SteeringAngleVD;
    unsigned int SAS_SteeringAngleSpd;
    float SAS_SteeringAngle;
    bool SAS_CalibratedSt;

    /************** GW_BCM_2_P_id 0x375 **********/
    __u8 BCM_KeySt;


    void GSM_1_decode(struct can_frame frame);
    void SAS_1_decode(struct can_frame frame);
    void GW_BCM_2_P_decode(struct can_frame frame);

    void ELSE_display(cv::Mat Display);

private:
    /* data */
};






























#endif