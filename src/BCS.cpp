#include <iostream>
#include "can/BCS.h"

BCS::BCS() {

    /**************** BCS_1 ********************/
    BCS_WheelSpd_resolution = 0.05625;   // km/h
    BCS_FLWheelSpdVD = false;
    BCS_FRWheelSpdVD = false;
    BCS_RLWheelSpdVD = false;
    BCS_RRWheelSpdVD = false;

    BCS_FLWheelSpd = 0;
    BCS_FRWheelSpd = 0;
    BCS_RLWheelSpd = 0;
    BCS_RRWheelSpd = 0;
 
    /****************** BCS_2 ******************/
    BCS_ABSActiveSt = false;
    BCS_ABSFaultSt = true;
    BCS_VDCFaultSt = true;
    BCS_VehSpdVD = false;
    BCS_VehSpd = 0;
    BCS_VDCActiveSt = false;
    BCS_BrkLightOn = false;

    /****************** BCS_3 *************************/

    BCS_ActVehLongAccelVD = false;
    BCS_ActVehLaltrlAccelVD = false;
    BCS_ActVehLongAccel = 0;
    BCS_VehLongAccelOffset = 0;
    BCS_ActVeLaltrlAccel = 0;
    BCS_VehLaltrlAccelOffset = 0;

    BCS_Veh_resolution = 0.027126736;   //单位m/s^2
    BCS_Veh_offset = -21.593;   

    /****************** BCS_5 ******************************/
    BCS_FLWheelSpdEdgesSum = 0;
    BCS_FRWheelSpdEdgesSum = 0;
    BCS_RLWheelSpdEdgesSum = 0;
    BCS_RRWheelSpdEdgesSum = 0;


    /******************* BCS_6 **********************************/
    BCS_YawRate = 0;          
    BCS_YawRateOffset = 0;    
    YawRate_resolution = 0.0021326;

    /****************** BCS_7 ********************************/
    BCS_HHCCtrlSt = false;
    BCS_HDCCtrlSt = 0;
    BCS_HHCErrSt = false;
    BCS_HDCErrSt = false;

    /****************** BCS_8 *********************************/
    BCS_MasterCylinderPrVD = false;
    BCS_MasterCylinderPrOffsetVD = false;
    BCS_MasterCylinderPr = 0;
    BCS_MasterCylinderPrOffset = 0;
    BCS_AEBActive = false;
    BCS_AEBAvailable = false;
    BCS_CDDActive = false;
    BCS_CDDAvailable = false;
    BCS_VehicleStandStillSt = 0;
    /****************** BCS_9 **********************************/
    BCS_FLWheelRotatedDirection = false;
    BCS_FLWheelRotatedDirectionVD = false;
    BCS_FRWheelRotatedDirectionVD = false;
    BCS_FRWheelRotatedDirection = false;
    BCS_9_MsgCounter = 0;
    BCS_9_Checksum = 0x55;
    /******************** BCS_10 ********************************/
    BCS_RLWheelRotatedDirection = false;
    BCS_RLWheelRotatedDirectionVD = false;
    BCS_RRWheelRotatedDirectionVD = false;
    BCS_RRWheelRotatedDirection = false;
    BCS_10_MsgCounter = 0;
    BCS_10_Checksum = 0x55;


}

BCS::~BCS() {
    cout << "~BCS()" << endl;
}

void BCS::BCS_1_decode(struct can_frame &frame) {

    __u16 WheelSpd_decode1 = 0x00;
    __u16 WheelSpd_decode2 = 0x00;
    BCS_FLWheelSpdVD = (frame.data[0] >> 7) & 0x01;
    BCS_FRWheelSpdVD = (frame.data[2] >> 5) & 0x01;
    BCS_RLWheelSpdVD = (frame.data[4] >> 5) & 0x01;
    BCS_RRWheelSpdVD = (frame.data[6] >> 5) & 0x01;

    //BCS_FLWheelSpd 
    WheelSpd_decode1 = (frame.data[0] >> 2) & 0x1F;    //高5位
    WheelSpd_decode2 = ((frame.data[0] << 6)& 0xC0) | ((frame.data[1] >> 2)& 0x3F);   //低8位
    WheelSpd_decode2 = (WheelSpd_decode1 << 8) | WheelSpd_decode2;    //组合成__u16，有效的是13位
    BCS_FLWheelSpd = WheelSpd_decode2 * BCS_WheelSpd_resolution;      //ofset = 0

    ////BCS_FRWheelSpd 
    WheelSpd_decode1 = frame.data[2] & 0x1F;    //高5位
    WheelSpd_decode2 = (WheelSpd_decode1 << 8) | frame.data[3];    //组合成__u16，有效的是13位
    BCS_FRWheelSpd = WheelSpd_decode2 * BCS_WheelSpd_resolution;      //ofset = 0

    //BCS_RLWheelSpd 
    WheelSpd_decode1 = frame.data[4] & 0x1F;    //高5位
    WheelSpd_decode2 = (WheelSpd_decode1 << 8) | frame.data[5];    //组合成__u16，有效的是13位
    BCS_RLWheelSpd = WheelSpd_decode2 * BCS_WheelSpd_resolution;      //ofset = 0

    //BCS_RRWheelSpd 
    WheelSpd_decode1 = frame.data[6] & 0x1F;    //高5位
    WheelSpd_decode2 = (WheelSpd_decode1 << 8) | frame.data[7];    //组合成__u16，有效的是13位
    BCS_RRWheelSpd = WheelSpd_decode2 * BCS_WheelSpd_resolution;      //ofset = 0

}


void BCS::BCS_2_decode(struct can_frame &frame) {

    __u16 VehSpd_decode1 = 0x00;
    __u16 VehSpd_decode2 = 0x00;
    BCS_ABSActiveSt = (frame.data[0] >> 1) & 0x01;
    BCS_ABSFaultSt = (frame.data[0] >> 2) & 0x01;
    BCS_VDCFaultSt = (frame.data[0] >> 7) & 0x01;
    BCS_VehSpdVD = (frame.data[4] >> 5) & 0x01;
    BCS_VDCActiveSt = (frame.data[6] >> 5) & 0x01;
    BCS_BrkLightOn = (frame.data[6] >> 6) & 0x01;

    VehSpd_decode1 = frame.data[4] & 0x1F;    //高5位
    VehSpd_decode2 = (VehSpd_decode1 << 8) | frame.data[5];    //组合成__u16，有效的是13位
    BCS_VehSpd = VehSpd_decode2 * BCS_WheelSpd_resolution;      //resolution = 0.05625, ofset = 0

}

void BCS::BCS_3_decode(struct can_frame &frame) {

    __u16 decode1 = 0x00;
    __u16 decode2 = 0x00;
    BCS_ActVehLongAccelVD = frame.data[0] & 0x01;
    BCS_ActVehLaltrlAccelVD = (frame.data[0] >> 1) & 0x01;
    //BCS_ActVehLongAccel
    decode1 = frame.data[1] & 0x0F;               //高4位
    decode2 = (decode1 << 8) | frame.data[2];     //低8位 
    BCS_ActVehLongAccel = decode2 * BCS_Veh_resolution + BCS_Veh_offset;

    //BCS_VehLongAccelOffset
    decode1 = (frame.data[3] >> 4) & 0x0F;
    decode2 = (frame.data[3] << 4) | (frame.data[4] >> 4);
    decode2 = (decode1 << 8) | decode2;
    BCS_VehLongAccelOffset = decode2 * BCS_Veh_resolution + BCS_Veh_offset;

    //BCS_ActVeLaltrlAccel
    decode1 = frame.data[4] & 0x0F;               //高4位
    decode2 = (decode1 << 8) | frame.data[5];     //低8位 
    BCS_ActVeLaltrlAccel = decode2 * BCS_Veh_resolution + BCS_Veh_offset;

    //BCS_VehLaltrlAccelOffset
    decode1 = frame.data[6] & 0x0F;               //高4位
    decode2 = (decode1 << 8) | frame.data[7];     //低8位 
    BCS_VehLaltrlAccelOffset = decode2 * BCS_Veh_resolution + BCS_Veh_offset;

}

void BCS::BCS_5_decode(struct can_frame &frame) {

    __u16 decode1 = 0x00;
    __u16 decode2 = 0x00;
    //BCS_FLWheelSpdEdgesSum
    decode1 = frame.data[0] >> 4;
    decode2 = (frame.data[0] << 4) | (frame.data[1] >> 4);
    decode2 = (decode1 << 8) | decode2;
    BCS_FLWheelSpdEdgesSum = decode2;       //resolution = 1, offset =0;

    //BCS_FRWheelSpdEdgesSum
    decode1 = frame.data[1] & 0x0F;
    decode2 = frame.data[2];
    decode2 = (decode1 << 8) | decode2;
    BCS_FRWheelSpdEdgesSum = decode2;       //resolution = 1, offset =0;

    //BCS_RLWheelSpdEdgesSum
    decode1 = frame.data[3] >> 4;
    decode2 = (frame.data[3] << 4) | (frame.data[4] >> 4);
    decode2 = (decode1 << 8) | decode2;
    BCS_RLWheelSpdEdgesSum = decode2;       //resolution = 1, offset =0;

    //BCS_RRWheelSpdEdgesSum
    decode1 = frame.data[4] & 0x0F;
    decode2 = frame.data[5];
    decode2 = (decode1 << 8) | decode2;
    BCS_RRWheelSpdEdgesSum = decode2;       //resolution = 1, offset =0;
}

void BCS::BCS_6_decode(struct can_frame &frame) {
    __u16 decode1 = 0x0000;
    __u16 decode2 = 0x0000;
    //BCS_YawRate
    decode1 = frame.data[0] & 0x0F;
    decode2 = frame.data[1];
    decode2 = (decode1 << 8) | decode2;
    BCS_YawRate = decode2 * YawRate_resolution - 2.0943;
    //BCS_YawRateOffset
    decode1 = frame.data[2] & 0x0F;
    decode2 = frame.data[3];
    decode2 = (decode1 << 8) | decode2;
    BCS_YawRateOffset = decode2 * YawRate_resolution - 0.13;
}

void BCS::BCS_7_decode(struct can_frame &frame) {
    BCS_HHCCtrlSt = (frame.data[2] >> 2) & 0x01;
    BCS_HDCCtrlSt = (frame.data[2] >> 3) & 0x03;
    BCS_HHCErrSt = (frame.data[2] >> 5) & 0x01;
    BCS_HDCErrSt = (frame.data[2] >> 6) & 0x01;
}

void BCS::BCS_8_decode(struct can_frame &frame) {
    __u16 decode1;
    __u16 decode2;
    BCS_MasterCylinderPrVD = (frame.data[0] >> 6) & 0x01;
    BCS_MasterCylinderPrOffsetVD = (frame.data[0] >> 7) & 0x01;

    //BCS_MasterCylinderPr
    decode1 = frame.data[0] & 0x0F;
    decode2 = frame.data[1];
    decode2 = (decode1 << 8) | decode2;
    BCS_MasterCylinderPr = decode2 *0.1;

    //BCS_MasterCylinderPrOffset
    decode1 = frame.data[2] & 0x03;
    decode2 = frame.data[3];
    decode2 = (decode1 << 8) | decode2;
    BCS_MasterCylinderPrOffset = decode2 * 0.1 - 15;

    BCS_AEBActive = (frame.data[4] >> 2) & 0x01;
    BCS_AEBAvailable = (frame.data[4] >> 3) & 0x01;
    BCS_CDDActive = (frame.data[4] >> 6) & 0x01;
    BCS_CDDAvailable = (frame.data[4] >> 7) & 0x01;

    //BCS_VehicleStandStillSt
    BCS_VehicleStandStillSt = (frame.data[5] >> 1) & 0x03;
}

void BCS::BCS_9_decode(struct can_frame &frame) {

    BCS_FLWheelRotatedDirection = (frame.data[0] >> 6) & 0x01;
    BCS_FLWheelRotatedDirectionVD = (frame.data[0] >> 7) & 0x01;
    BCS_FRWheelRotatedDirectionVD = (frame.data[3] >> 5) & 0x01;
    BCS_FRWheelRotatedDirection = (frame.data[3] >> 6) & 0x01;
    BCS_9_MsgCounter = frame.data[6] & 0x0F;
    BCS_9_Checksum = frame.data[7];
}

void BCS::BCS_10_decode(struct can_frame &frame) {

    BCS_RLWheelRotatedDirection = (frame.data[0] >> 6) & 0x01;
    BCS_RLWheelRotatedDirectionVD = (frame.data[0] >> 7) & 0x01;
    BCS_RRWheelRotatedDirectionVD = (frame.data[3] >> 5) & 0x01;
    BCS_RRWheelRotatedDirection = (frame.data[3] >> 6) & 0x01;
    BCS_10_MsgCounter = frame.data[6] & 0x0F;                     //-resolution: 1 per bit   -offset:0
    BCS_10_Checksum = frame.data[7];
}



/***************************** display **************************************/

void BCS::BCS_display(cv::Mat Display) {

    char title_data[21][40];
    char title_data2[31][50];

    //BCS_1
    if(BCS_FLWheelSpdVD)
        sprintf(title_data[0], "BCS_FLWheelSpdVD : Valid");
    else
        sprintf(title_data[0], "BCS_FLWheelSpdVD : Not Valid");
    if(BCS_FRWheelSpdVD)
        sprintf(title_data[1], "BCS_FRWheelSpdVD : Valid");
    else
        sprintf(title_data[1], "BCS_FRWheelSpdVD : Not Valid");
    if(BCS_RLWheelSpdVD)
        sprintf(title_data[2], "BCS_RLWheelSpdVD : Valid");
    else
        sprintf(title_data[2], "BCS_RLWheelSpdVD : Not Valid");
    if(BCS_RRWheelSpdVD)
        sprintf(title_data[3], "BCS_RRWheelSpdVD : Valid");
    else
        sprintf(title_data[3], "BCS_RRWheelSpdVD : Not Valid");

    if(BCS_FLWheelSpd > 240 || BCS_FLWheelSpd < 0)
        sprintf(title_data[4], "BCS_FLWheelSpd : error");
    else
        sprintf(title_data[4], "BCS_FLWheelSpd : %.5f", BCS_FLWheelSpd);
    if(BCS_FRWheelSpd > 240 || BCS_FRWheelSpd < 0)
        sprintf(title_data[5], "BCS_FRWheelSpd : error");
    else
        sprintf(title_data[5], "BCS_FRWheelSpd : %.5f", BCS_FRWheelSpd);

    if(BCS_RLWheelSpd > 240 || BCS_RLWheelSpd < 0)
        sprintf(title_data[6], "BCS_RLWheelSpd : error");
    else
        sprintf(title_data[6], "BCS_RLWheelSpd : %.5f", BCS_RLWheelSpd);
    if(BCS_RRWheelSpd > 240 || BCS_RRWheelSpd < 0)
        sprintf(title_data[7], "BCS_RRWheelSpd : error");
    else
        sprintf(title_data[7], "BCS_RRWheelSpd : %.5f", BCS_RRWheelSpd);
    
    //BCS_2
    if(BCS_ABSActiveSt)
        sprintf(title_data[8], "BCS_ABSActiveSt : Active");
    else
        sprintf(title_data[8], "BCS_ABSActiveSt : Not active");
    if(BCS_ABSFaultSt)
        sprintf(title_data[9], "BCS_ABSFaultSt : Fault");
    else
        sprintf(title_data[9], "BCS_ABSFaultSt : Not Fault");
    if(BCS_VDCFaultSt)
        sprintf(title_data[10], "BCS_VDCFaultSt : Fault");
    else
        sprintf(title_data[10], "BCS_VDCFaultSt : Not  Fault");
    if(BCS_VehSpdVD)
        sprintf(title_data[11], "BCS_VehSpdVD : Valid");
    else
        sprintf(title_data[11], "BCS_VehSpdVD : Not Valid");

    if(BCS_VehSpd > 240 || BCS_VehSpd < 0)
        sprintf(title_data[12], "BCS_VehSpd : error");
    else
        sprintf(title_data[12], "BCS_VehSpd : %.5f", BCS_VehSpd);
    if(BCS_VDCActiveSt)
        sprintf(title_data[13], "BCS_VDCActiveSt : Active");
    else
        sprintf(title_data[13], "BCS_VDCActiveSt : Not Active");
    if(BCS_BrkLightOn)
        sprintf(title_data[14], "BCS_BrkLightOn : On");
    else
        sprintf(title_data[14], "BCS_BrkLightOn : Off");
    
    //BCS_3
    if(BCS_ActVehLongAccelVD)
        sprintf(title_data[15], "BCS_ActVehLongAccelVD : Valid");
    else
        sprintf(title_data[15], "BCS_ActVehLongAccelVD : Not Valid");
    if(BCS_ActVehLaltrlAccelVD)
        sprintf(title_data[16], "BCS_ActVehLaltrlAccelVD : Valid");
    else
        sprintf(title_data[16], "BCS_ActVehLaltrlAccelVD : Not Valid");

    if(BCS_ActVehLongAccel > 21.593 || BCS_VehSpd < -21.6)
        sprintf(title_data[17], "BCS_ActVehLongAccel : error");
    else
        sprintf(title_data[17], "BCS_ActVehLongAccel : %.5f", BCS_ActVehLongAccel);

    if(BCS_VehLongAccelOffset > 21.593 || BCS_VehSpd < -21.6)
        sprintf(title_data[18], "BCS_VehLongAccelOffset : error");
    else
        sprintf(title_data[18], "BCS_VehLongAccelOffset : %.5f", BCS_VehLongAccelOffset);

    if(BCS_ActVeLaltrlAccel > 21.593 || BCS_VehSpd < -21.6)
        sprintf(title_data[19], "BCS_ActVeLaltrlAccel : error");
    else
        sprintf(title_data[19], "BCS_ActVeLaltrlAccel : %.5f", BCS_ActVeLaltrlAccel);

    if(BCS_VehLaltrlAccelOffset > 21.593 || BCS_VehSpd < -21.6)
        sprintf(title_data[20], "BCS_VehLaltrlAccelOffset : error");
    else
        sprintf(title_data[20], "BCS_VehLaltrlAccelOffset : %.5f", BCS_VehLaltrlAccelOffset);

    //BCS_5
    sprintf(title_data2[0], "BCS_FLWheelSpdEdgesSum : %d", BCS_FLWheelSpdEdgesSum);
    sprintf(title_data2[1], "BCS_FRWheelSpdEdgesSum : %d", BCS_FRWheelSpdEdgesSum);
    sprintf(title_data2[2], "BCS_RLWheelSpdEdgesSum : %d", BCS_RLWheelSpdEdgesSum);
    sprintf(title_data2[3], "BCS_RRWheelSpdEdgesSum : %d", BCS_RRWheelSpdEdgesSum);

    //BCS_6
    if(BCS_YawRate > 2.0943 || BCS_VehSpd < -2.09)
        sprintf(title_data2[4], "BCS_YawRate : error");
    else
        sprintf(title_data2[4], "BCS_YawRate : %.7f", BCS_YawRate);

    if(BCS_YawRateOffset > 0.13 || BCS_VehSpd < -0.13)
        sprintf(title_data2[5], "BCS_YawRateOffset : error");
    else
        sprintf(title_data2[5], "BCS_YawRateOffset : %.7f", BCS_YawRateOffset);
    
    //BCS_7
    if(BCS_HHCCtrlSt)
        sprintf(title_data2[6], "BCS_HHCCtrlSt : Active");
    else
        sprintf(title_data2[6], "BCS_HHCCtrlSt : Not active");
    switch(BCS_HDCCtrlSt) {
        case 0:
            sprintf(title_data2[7], "BCS_HDCCtrlSt : Off");
            break;
        case 1:
            sprintf(title_data2[7], "BCS_HDCCtrlSt : On active braking");
            break;
        case 2:
            sprintf(title_data2[7], "BCS_HDCCtrlSt : On not active braking");
            break;
        case 3:
            sprintf(title_data2[7], "BCS_HDCCtrlSt : Not usBCS_10_MsgCountered");
            break;
        default:
            break;
    }

    if(BCS_HHCErrSt)
        sprintf(title_data2[8], "BCS_HHCErrSt : Error");
    else
        sprintf(title_data2[8], "BCS_HHCErrSt : No error");

    if(BCS_HDCErrSt)
        sprintf(title_data2[9], "BCS_HDCErrSt : Error");
    else
        sprintf(title_data2[9], "BCS_HDCErrSt : No error");
    
    //BCS_8
    if(BCS_MasterCylinderPrVD)
        sprintf(title_data2[10], "BCS_MasterCylinderPrVD : Valid");
    else
        sprintf(title_data2[10], "BCS_MasterCylinderPrVD : Invalid");

    if(BCS_MasterCylinderPrOffsetVD)
        sprintf(title_data2[11], "BCS_MasterCylinderPrOffsetVD : Valid");
    else
        sprintf(title_data2[11], "BCS_MasterCylinderPrOffsetVD : Invalid");

    if(BCS_MasterCylinderPr > 250 || BCS_VehSpd < 0)
        sprintf(title_data2[12], "BCS_MasterCylinderPr : error");
    else
        sprintf(title_data2[12], "BCS_MasterCylinderPr : %.1f", BCS_MasterCylinderPr);

    if(BCS_MasterCylinderPrOffset > 15 || BCS_MasterCylinderPrOffset < -15)
        sprintf(title_data2[13], "BCS_MasterCylinderPrOffset : error");
    else
        sprintf(title_data2[13], "BCS_MasterCylinderPrOffset : %.1f", BCS_MasterCylinderPrOffset);
    
    if(BCS_AEBActive)
        sprintf(title_data2[14], "BCS_AEBActive : Active");
    else
        sprintf(title_data2[14], "BCS_AEBActive : Not active");

    if(BCS_AEBAvailable)
        sprintf(title_data2[15], "BCS_AEBAvailable : Available");
    else
        sprintf(title_data2[15], "BCS_AEBAvailable : Not available");
    
    if(BCS_CDDActive)
        sprintf(title_data2[16], "BCS_CDDActive : Active");
    else
        sprintf(title_data2[16], "BCS_CDDActive : Not active");
    
    if(BCS_CDDAvailable)
        sprintf(title_data2[17], "BCS_CDDAvailable : Available");
    else
        sprintf(title_data2[17], "BCS_CDDAvailable : Not available");

    switch(BCS_VehicleStandStillSt) {
        case 0:
            sprintf(title_data2[18], "BCS_VehicleStandStillSt : Not standstill");
            break;
        case 1:
            sprintf(title_data2[18], "BCS_VehicleStandStillSt : Standstill");
            break;
        case 2:
            sprintf(title_data2[18], "BCS_VehicleStandStillSt : Invalid");
            break;
        case 3:
            sprintf(title_data2[18], "BCS_VehicleStandStillSt : Not used");
            break;
        default:
            break;
    }

    //BCS_9
    if(BCS_FLWheelRotatedDirection)
        sprintf(title_data2[19], "BCS_FLWheelRotatedDirection : Backward");
    else
        sprintf(title_data2[19], "BCS_FLWheelRotatedDirection : Forward");

    if(BCS_FLWheelRotatedDirectionVD)
        sprintf(title_data2[20], "BCS_FLWheelRotatedDirectionVD : Valid");
    else
        sprintf(title_data2[20], "BCS_FLWheelRotatedDirectionVD : Invalid");
    
    if(BCS_FRWheelRotatedDirectionVD)
        sprintf(title_data2[21], "BCS_FRWheelRotatedDirectionVD : Valid");
    else
        sprintf(title_data2[21], "BCS_FRWheelRotatedDirectionVD : Invalid");
    if(BCS_FRWheelRotatedDirection)
        sprintf(title_data2[22], "BCS_FRWheelRotatedDirection : Backward");
    else
        sprintf(title_data2[22], "BCS_FRWheelRotatedDirection : Forward");

    if(BCS_9_MsgCounter > 15 || BCS_9_MsgCounter < 0)
        sprintf(title_data2[23], "BCS_9_MsgCounter : error");
    else
        sprintf(title_data2[23], "BCS_9_MsgCounter : %d", BCS_9_MsgCounter);
    
    if(BCS_9_Checksum > 255 || BCS_9_Checksum < 0)
        sprintf(title_data2[24], "BCS_9_Checksum : error");
    else
        sprintf(title_data2[24], "BCS_9_Checksum : %d", BCS_9_Checksum);
    
    //BCS_10
    if(BCS_RLWheelRotatedDirection)
        sprintf(title_data2[25], "BCS_RLWheelRotatedDirection : Backward");
    else
        sprintf(title_data2[25], "BCS_RLWheelRotatedDirection : Forward");

    if(BCS_RLWheelRotatedDirectionVD)
        sprintf(title_data2[26], "BCS_RLWheelRotatedDirectionVD : Valid");
    else
        sprintf(title_data2[26], "BCS_RLWheelRotatedDirectionVD : Invalid");
    
    if(BCS_RRWheelRotatedDirectionVD)
        sprintf(title_data2[27], "BCS_RRWheelRotatedDirectionVD : Valid");
    else
        sprintf(title_data2[27], "BCS_RRWheelRotatedDirectionVD : Invalid");
    if(BCS_RRWheelRotatedDirection)
        sprintf(title_data2[28], "BCS_RRWheelRotatedDirection : Backward");
    else
        sprintf(title_data2[28], "BCS_RRWheelRotatedDirection : Forward");

    if(BCS_10_MsgCounter > 15 || BCS_10_MsgCounter < 0)
        sprintf(title_data2[29], "BCS_10_MsgCounter : error");
    else
        sprintf(title_data2[29], "BCS_10_MsgCounter : %d", BCS_9_MsgCounter);
    
    if(BCS_10_Checksum > 255 || BCS_10_Checksum < 0)
        sprintf(title_data2[30], "BCS_10_Checksum : error");
    else
        sprintf(title_data2[30], "BCS_10_Checksum : %d", BCS_10_Checksum);

    

    for (int i = 0; i<8 ;i++) {
        putText(Display, title_data[i], Point(50,300+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }
    for (int i = 8; i<15 ;i++) {
        putText(Display, title_data[i], Point(50,350+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }

    for (int i = 15; i<21 ;i++) {
        putText(Display, title_data[i], Point(50,400+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }

    for (int i = 0; i<4 ;i++) {
        putText(Display, title_data2[i], Point(50,1450+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }

    for (int i = 4; i<10 ;i++) {
        putText(Display, title_data2[i], Point(700,-100+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }

    for (int i = 10; i<19 ;i++) {
        putText(Display, title_data2[i], Point(700,-50+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }
    
    for (int i = 19; i<25 ;i++) {
        putText(Display, title_data2[i], Point(700, 50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }

    for (int i = 25; i<31 ;i++) {
        putText(Display, title_data2[i], Point(700,50+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }
}
