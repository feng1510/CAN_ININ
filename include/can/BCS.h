#ifndef BCS_H
#define BSC_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>
#include "define.h"

using namespace std;
using namespace cv;



class BCS
{
public:
    BCS();
    virtual ~BCS();

    /***************************** BCS_1   0x36B **************************************/
    double BCS_WheelSpd_resolution;
    bool BCS_FLWheelSpdVD;
    double BCS_FLWheelSpd;
    bool BCS_FRWheelSpdVD;
    double BCS_FRWheelSpd;
    bool BCS_RLWheelSpdVD;
    double BCS_RLWheelSpd;
    bool BCS_RRWheelSpdVD;
    double BCS_RRWheelSpd;

    /***************************** BCS_2  0x260 ***************************************************/

    bool BCS_ABSActiveSt;
    bool BCS_ABSFaultSt;
    bool BCS_VDCFaultSt;
    bool BCS_VehSpdVD;
    double BCS_VehSpd;
    bool BCS_VDCActiveSt;
    bool BCS_BrkLightOn;

    /**************************** BCS_3 0x268 *******************************************************/

    bool BCS_ActVehLongAccelVD;    //纵向
    bool BCS_ActVehLaltrlAccelVD;  //横向
    double BCS_ActVehLongAccel;
    double BCS_VehLongAccelOffset;
    double BCS_ActVeLaltrlAccel;
    double BCS_VehLaltrlAccelOffset;

    double BCS_Veh_resolution;
    double BCS_Veh_offset;

    /******************************* BCS_5_id 0x36C **********************************/
    //-resolution: 1per bit
    //-offset:0
    int BCS_FLWheelSpdEdgesSum;
    int BCS_FRWheelSpdEdgesSum;
    int BCS_RLWheelSpdEdgesSum;
    int BCS_RRWheelSpdEdgesSum;

    /******************************* BCS_6_id 0x3B6 **********************************/
    double BCS_YawRate;          //offset = -2.0943 
    double BCS_YawRateOffset;    //offset = -0.13
    double YawRate_resolution;   //0.0021326rad/s

    /*************************** BCS_7_id 0x294 *************************************/
    bool BCS_HHCCtrlSt;
    unsigned char BCS_HDCCtrlSt;
    bool BCS_HHCErrSt;
    bool BCS_HDCErrSt;

    /**************************** BCS_8_id 0x28C*************************************/
    bool BCS_MasterCylinderPrVD;
    bool BCS_MasterCylinderPrOffsetVD;
    float BCS_MasterCylinderPr;                //-resolution: 0.1bar per bit    -offset:0
    float BCS_MasterCylinderPrOffset;          //-resolution: 0.1bar per bit    -offset: -15
    bool BCS_AEBActive;
    bool BCS_AEBAvailable;
    bool BCS_CDDActive;
    bool BCS_CDDAvailable;
    unsigned char BCS_VehicleStandStillSt;
                                                // 0=Not standstill
                                                // 1=standstill
                                                // 2=Invalid (short unavailability, max 3s)
                                                // 3=Not used

    /***************************** BCS_9_id 0x26C ***********************************/
    bool BCS_FLWheelRotatedDirection;
    bool BCS_FLWheelRotatedDirectionVD;
    bool BCS_FRWheelRotatedDirectionVD;
    bool BCS_FRWheelRotatedDirection;
    __u8 BCS_9_MsgCounter;           //-resolution: 1 per bit   -offset:0
    __u8 BCS_9_Checksum;

    /**************************** BCS_10_id 0x26D ***********************************/
    bool BCS_RLWheelRotatedDirection;
    bool BCS_RLWheelRotatedDirectionVD;
    bool BCS_RRWheelRotatedDirectionVD;
    bool BCS_RRWheelRotatedDirection;
    __u8 BCS_10_MsgCounter;           //-resolution: 1 per bit   -offset:0
    __u8 BCS_10_Checksum;







    /***************************************** decode ********************************************************/
    void BCS_1_decode(struct can_frame &frame);
    void BCS_2_decode(struct can_frame &frame);
    void BCS_3_decode(struct can_frame &frame);
    void BCS_5_decode(struct can_frame &frame);
    void BCS_6_decode(struct can_frame &frame);
    void BCS_7_decode(struct can_frame &frame);
    void BCS_8_decode(struct can_frame &frame);
    void BCS_9_decode(struct can_frame &frame);
    void BCS_10_decode(struct can_frame &frame);
    
    /******************************************* display **********************************************/
    void BCS_display(cv::Mat Display);




private:
    /* data */
};








#endif