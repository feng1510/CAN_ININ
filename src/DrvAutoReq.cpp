#include <iostream>
#include "can/DrvAutoReq.h"

DrvAutoReq::DrvAutoReq() {

    cout << "**Auto Drv Req**" << endl;

    DrvAutoReq_send_flag = false;
    DrvAuto_flag = 0;


}


DrvAutoReq::~DrvAutoReq() {
    cout << "~DrvAutoReq()" << endl;
}