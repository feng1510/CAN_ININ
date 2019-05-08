#include <in2_control/lateral_control.h>

LATERAL::LATERAL(double Strategy_[5][5], double X1_[5], double X2_[5])
{
    cout<<"LATERAL::INIT"<<endl;

    out_angle = 50.0;
    out_speedgoal = 0.0;

    DFar = 0.0;
    kp_angle = 1.0;
    kp_position = 1.0;
    qianzhan_angle = 6.0;//meter
    qianzhan_position = 1.0;//meter
    speedK = 100.0;
    
    err_angle = 0;
    err_position = 0;
    IndexOfControlPoint_angle = 0;
    IndexOfCarCenter = 0;
    IndexOfControlPoint_position = 0;
    IndexOfFarPoint = 300;

    Strategy = new double*[5];
    X1 = new double[5];
    X2 = new double[5];
    for(int i = 0; i<5; i++)
       Strategy[i] = new double[5];
    for(int i = 0; i<5; i++)
       for(int j = 0; j<5; j++)
          Strategy[i][j] = Strategy_[i][j];
    for(int i = 0; i<5; i++)
          X1[i] = X1_[i];
    for(int i = 0; i<5; i++)
          X2[i] = X2_[i];
}

double LATERAL::Fuzzy(double Feedback1, double Feedback2){
    double* A1 = new double[5]{0,0,0,0,0};
    double* A2 = new double[5]{0,0,0,0,0};
 
    if (Feedback1 < X1[0]){ 
       A1[0] = 1; 
       A1[1] = 0;
       A1[2] = 0;
       A1[3] = 0;
       A1[4] = 0;
    }
    else if (Feedback1 < X1[1]){
       A1[0] = TwoPointLine(X1[1], X1[0], Feedback1);
       A1[1] = TwoPointLine(X1[0], X1[1], Feedback1);
       A1[2] = 0;
       A1[3] = 0;
       A1[4] = 0;
    }
    else if (Feedback1 < X1[2]){
       A1[0] = 0;
       A1[1] = TwoPointLine(X1[2], X1[1], Feedback1);
       A1[2] = TwoPointLine(X1[1], X1[2], Feedback1);
       A1[3] = 0;
       A1[4] = 0;
    }
    else if (Feedback1 < X1[3]){
       A1[0] = 0;
       A1[1] = 0;
       A1[2] = TwoPointLine(X1[3], X1[2], Feedback1);
       A1[3] = TwoPointLine(X1[2], X1[3], Feedback1);
       A1[4] = 0;
    }
    else if (Feedback1 < X1[4]){
       A1[0] = 0;
       A1[1] = 0;
       A1[2] = 0;
       A1[3] = TwoPointLine(X1[4], X1[3], Feedback1);
       A1[4] = TwoPointLine(X1[3], X1[4], Feedback1);
     }
     else{
       A1[0] = 0;
       A1[1] = 0;
       A1[2] = 0;
       A1[3] = 0;
       A1[4] = 1;
     }

     if (Feedback2 < X2[0]){
       A2[0] = 1;
       A2[1] = 0;
       A2[2] = 0;
       A2[3] = 0;
       A2[4] = 0;
     }
     else if (Feedback2 < X2[1]){
       A2[0] = TwoPointLine(X2[1], X2[0], Feedback2);
       A2[1] = TwoPointLine(X2[0], X2[1], Feedback2);
       A2[2] = 0;
       A2[3] = 0;
       A2[4] = 0;
     }
     else if (Feedback2< X2[2]){
       A2[0] = 0;
       A2[1] = TwoPointLine(X2[2], X2[1], Feedback2);
       A2[2] = TwoPointLine(X2[1], X2[2], Feedback2);
       A2[3] = 0;
       A2[4] = 0;
     }
     else if (Feedback2 < X2[3]){
       A2[0] = 0;
       A2[1] = 0;
       A2[2] = TwoPointLine(X2[3], X2[2], Feedback2);
       A2[3] = TwoPointLine(X2[2], X2[3], Feedback2);
       A2[4] = 0;
     }
     else if (Feedback2 < X2[4]){
       A2[0] = 0;
       A2[1] = 0;
       A2[2] = 0;
       A2[3] = TwoPointLine(X2[4], X2[3], Feedback2);
       A2[4] = TwoPointLine(X2[3], X2[4], Feedback2);
     }
     else{
       A2[0] = 0;
       A2[1] = 0;
       A2[2] = 0;
       A2[3] = 0;
       A2[4] = 1;
    }
    double Sum = 0;
    for (int i = 0; i < 5; i++)
       for (int j = 0; j < 5; j++)
          Sum += A1[i] * A2[j] * Strategy[i][j];
    delete[] A1;
    delete[] A2;
    return Sum;
}

double LATERAL::TwoPointLine(double x0, double x1, double x){
     double y = (x - x0) / (x1 - x0);
     return y;
}

void LATERAL::compute(vector<Point2d>& roadpoints, double SpeedLimit, double SpeedNow){
    
    IndexOfControlPoint_angle = 0;
    IndexOfCarCenter = 0;
    IndexOfControlPoint_position = 0;
    IndexOfFarPoint = 300;
    
    qianzhan_angle = SpeedNow/40.0 * 32.0;
    if(qianzhan_angle<8) qianzhan_angle = 8;
    if(qianzhan_angle>16) qianzhan_angle = 16;
    
    //int step = 0;
    double distance = 0;
    double distance_min = 1000.0;
    double distance_max = 0.0;
    double distance_sum = 0.0;
    for (int i = 0; i < roadpoints.size(); i++){
       distance = sqrt(roadpoints[i].x * roadpoints[i].x + roadpoints[i].y * roadpoints[i].y);
       if (distance < distance_min){
          IndexOfCarCenter = i;
          distance_min = distance;
       }
    }
    for (int i = IndexOfCarCenter; i < 300; i++){
       double dx = roadpoints[i + 1].x - roadpoints[i].x;
       double dy = roadpoints[i + 1].y - roadpoints[i].y;
       distance = sqrt(dx * dx + dy * dy);
       distance_sum += distance;
       if (distance_sum >= qianzhan_angle && IndexOfControlPoint_angle == 0) IndexOfControlPoint_angle = i;
       if (distance_sum >= qianzhan_position && IndexOfControlPoint_position == 0) IndexOfControlPoint_position = i;

       distance = sqrt(roadpoints[i].x * roadpoints[i].x + roadpoints[i].y * roadpoints[i].y);
       if (distance > distance_max){
          IndexOfFarPoint = i;
          distance_max = distance;
       }
    }
    if (IndexOfControlPoint_angle == 0) IndexOfControlPoint_angle = 300;
    if (IndexOfControlPoint_position == 0) IndexOfControlPoint_position = 300;

    double disX = roadpoints[IndexOfControlPoint_angle].x - roadpoints[IndexOfCarCenter].x;
    double disY = roadpoints[IndexOfControlPoint_angle].y - roadpoints[IndexOfCarCenter].y;
    if ( (disX>0) && (disY>0) ){
       err_angle = 180.0 * (atan(disX / disY)) / D_PI;
    }
    else if ((disX < 0) && (disY > 0)){
       err_angle = 180.0 * (atan(disX / disY)) / D_PI;
    }
    else if ((disX < 0) && (disY < 0)){
       err_angle = 180.0 * (atan(disX / disY)) / D_PI;
       err_angle = err_angle - 180.0;
    }
    else if ((disX > 0) && (disY < 0)){
       err_angle = 180.0 *(atan(disX / disY)) / D_PI;
       err_angle = err_angle + 180.0;
    }
    else if ((disX == 0) && (disY > 0)){
       err_angle = 0;
    }
    else if ((disX == 0) && (disY < 0)){
       if(roadpoints[IndexOfControlPoint_angle].x >= 0)
          err_angle = 180.0;
       else
          err_angle = -180.0;
    }
    else if ((disX > 0) && (disY == 0)){
          err_angle = 90.0;
    }
    else if ((disX < 0) && (disY == 0)){
          err_angle = -90.0;
    }
    else{
          err_angle = 0.0;
    }

    for (int i = IndexOfCarCenter; i <= IndexOfControlPoint_position; i++){
          err_position += roadpoints[i].x;
    }
    if ((1 + IndexOfControlPoint_position - IndexOfCarCenter) != 0){
          err_position = err_position / (1 + IndexOfControlPoint_position - IndexOfCarCenter);
    }
    else{
          err_position = 0;
    }

    double Kp_angle_f = kp_angle * (60.0 - fabs(SpeedNow)) / 60.0;
    double Kp_position_f = kp_position * (40.0 - fabs(SpeedNow)) / 40.0;
    if (Kp_angle_f < 0.3) Kp_angle_f = 0.3;
    if (Kp_position_f < 0.0) Kp_position_f = 0.0;
    
    if (SpeedLimit > 0.0){
       out_angle = 50.0 + Kp_angle_f * err_angle + Kp_position_f * err_position;
    }
    else if (SpeedLimit < 0.0){
       if (err_angle > 0){
          err_angle = 180.0 - err_angle;
       }
       else if (err_angle < 0){
          err_angle = -180.0 - err_angle;
       }
       out_angle = 50.0 + kp_angle * err_angle + kp_position * err_position;
     }
     else{
       out_angle = 50.0;
     }

     if (out_angle > 100.0){
       out_angle = 100.0;
     }
     if (out_angle < 0.0){
       out_angle = 0.0;
     }

     DFar = roadpoints[IndexOfFarPoint].y;
     out_speedgoal = speedK / 100.0 * Fuzzy(fabs(out_angle-50.0),roadpoints[IndexOfFarPoint].y);
     if (SpeedLimit > 0.0){
        out_speedgoal = speedK / 100.0 * Fuzzy(fabs(out_angle-50.0),roadpoints[IndexOfFarPoint].y);
     }
     else if (SpeedLimit < 0.0){
        out_speedgoal = -speedK / 100.0 * Fuzzy(fabs(out_angle-50.0),-roadpoints[IndexOfFarPoint].y);
     }
     else{
        out_speedgoal = 0.0;
     }
     if(abs(out_speedgoal)>abs(SpeedLimit)) out_speedgoal = SpeedLimit;
     out_angle = 100 - out_angle;
}
