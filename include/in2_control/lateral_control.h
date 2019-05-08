#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <common/TcpGeneral.h>
#define D_PI 3.1415926536f

using namespace std;
using namespace cv;

class LATERAL
{
public:
    LATERAL(double Strategy_[5][5], double X1_[5], double X2_[5]);
    
public:
    
    double err_angle;
    double err_position;
    double out_angle;
    double out_speedgoal;
    double DFar;

    double** Strategy;
    double* X1;
    double* X2;

    double kp_angle;
    double kp_position;
    double qianzhan_angle;//meter
    double qianzhan_position;//meter
    double speedK;

    int IndexOfControlPoint_angle;
    int IndexOfCarCenter;
    int IndexOfControlPoint_position;
    int IndexOfFarPoint;
    
    double TwoPointLine(double x0, double x1, double x);
    
public:    
    void compute(vector<Point2d> &roadpoints, double SpeedLimit, double SpeedNow);
    double Fuzzy(double Feedback1, double Feedback2);

};
