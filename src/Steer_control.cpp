#include <iostream>
#include "can/Steer_control.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>


using namespace std;
using namespace cv;


Steer_control::Steer_control() {

    SCU_EpsOnReq = false;
    SCU_SteerAngReq = 0;

    angle_input = 0;

    frame.can_id = SCU_2;
    frame.can_dlc = 8;
	frame.data[0] = 0x0;
	frame.data[1] = 0x1E;
	frame.data[2] = 0x78;
	frame.data[3] = 0x0;
	frame.data[4] = 0x0;
	frame.data[5] = 0x0;
	frame.data[6] = 0x0;
	frame.data[7] = 0x0;
}

Steer_control::~Steer_control() {
    cout << "~Steer_control" << endl;
}

void Steer_control::steer_control_code() {

        if(angle_input > 779)
            angle_input = 779;
        else if(angle_input < -780)
            angle_input = -780;
        
        int16_t angle_code = (angle_input + 780) * 10;    //偏置-780, 精度0.1;
        
        frame.data[1] = (angle_code >> 8) & 0xFF;
        frame.data[2] = angle_code & 0xFF;
}

void Steer_control::in_steer_auto(EPS& eps) {
    if(eps.EPS_ControlMode == 1 && eps.EPS_FailureSt == false){
        frame.data[0] = 0x80;
        frame.data[1] = 0x1E;
        frame.data[2] = 0x78;
    }
}

void Steer_control::esc_steer_auto() {
        frame.data[0] = 0x00;
	    frame.data[1] = 0x1E;
	    frame.data[2] = 0x78;
}

void Steer_control::steer_loop_decode(struct can_frame &frame){
    int16_t decode1 = 0x00;
    int16_t decode2 = 0x00;
    SCU_EpsOnReq = (frame.data[0] >> 7) & 0x01;
    decode1 = frame.data[1];
    decode2 = frame.data[2];
    decode2 = (decode1 << 8) | decode2;
    SCU_SteerAngReq = decode2 * 0.1 - 780;
}



void Steer_control::steer_control_dis(EPS& eps, cv::Mat Display_control){
    char title_data[5][50];
    switch(eps.EPS_ControlMode) {
        case 0:
            sprintf(title_data[0],"EPS_ControlMode : Idle");
            break;
        case 1:
            sprintf(title_data[0],"EPS_ControlMode : Manual Assist");
            break;
        case 2:
            sprintf(title_data[0],"EPS_ControlMode : Auto Steering");
            break;
        default:
            sprintf(title_data[0],"**ERROR:EPS_ControlMode : decode error**");
            break;
    }

    if(eps.EPS_FailureSt) 
        sprintf(title_data[1],"EPS_FailureSt : fail");
    else
        sprintf(title_data[1],"EPS_FailureSt : not fail");

    sprintf(title_data[2],"EPS_SteeringAngle: %.1f",eps.EPS_SteeringAngle);

    if(SCU_EpsOnReq) 
        sprintf(title_data[3],"SCU_EpsOnReq : active");
    else
        sprintf(title_data[3],"SCU_EpsOnReq : Not active");

    sprintf(title_data[4],"SCU_SteerAngReq: %.1f", SCU_SteerAngReq);

    for(int i = 0; i<3; i++)
        putText(Display_control, title_data[i], Point(2400, 500+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    for(int i = 3; i<5; i++)
        putText(Display_control, title_data[i], Point(2400, 500+50*i), fontFace, fontScale,CV_RGB(0,0,255) , 2*thickness);
}