#ifndef DRVAUTOREQ_H
#define DRVAUTOREQ_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>

using namespace std;
using namespace cv;

class DrvAutoReq
{
public:
    DrvAutoReq();
    virtual ~DrvAutoReq();   //析构函数

    bool DrvAutoReq_send_flag;
    int16_t  DrvAuto_flag;
    


private:
    /* data */
};











#endif