#ifndef DEFINE_H
#define DEFINE_H
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>



#define EPS_1_id 0x366
#define EPS_11_id 0x365
#define EPB_1_id 0x397

#define BCS_1_id 0x36B
#define BCS_2_id 0x260
#define BCS_3_id 0x268
#define BCS_5_id 0x36C
#define BCS_6_id 0x3B6
#define BCS_7_id 0x294
#define BCS_8_id 0x28C
#define BCS_9_id 0x26C
#define BCS_10_id 0x26D

#define VCU_2_P_id 0x360
#define VCU_7_P_id 0x39F
#define VCU_9_P_id 0x2AB
#define VCU_18_P_id 0x37C
#define VCU_19_P_id 0x37D
#define VCU_20_id 0x186
#define VCU_21_id 0x187
#define VCU_EMS_1_id 0x18D
#define VCU_EMS_6_id 0x279

#define GSM_1_id 0x190
#define SAS_1_id 0x264
#define GW_BCM_2_P_id 0x375


//SCU发送
#define SCU_1 0x37A    //驱动控制
#define SCU_2 0x37B    //转向控制
#define SCU_3 0x384    //电子驻车






#define D_PI 3.1415926536f
#define fontFace FONT_HERSHEY_SIMPLEX
#define fontScale 1.0
#define thickness 1

#define Valid true
#define Invalid false
#define Forward false
#define Backward true
#define Active true
#define Not_active false
#define Available true
#define Not_available false
#define Error true
#define No_error false


#endif