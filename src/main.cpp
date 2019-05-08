#include <stdio.h>
#include <iostream>
#include <time.h>
#include <ros/ros.h>
#include <opencv/highgui.h>
#include <common/TcpGeneral.h>
#include <in2_control/lateral_control.h>
#include <remote_car/car_data.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <fcntl.h>

#include <common/VehicleInfo.h>

#include <can/EPS.h>
#include <can/BCS.h>
#include <can/VCU.h>
#include <can/ELSE.h>
#include <can/Steer_control.h>
#include <can/DrvAutoReq.h>

using namespace std;

ros::Publisher vehicle_info;  //---publisher
char* can2_name = (char*)"can2";
//char can2_name[] = "can2";
EPS eps;
BCS bcs;
VCU vcu;
ELSE elseid;
Steer_control steer_control;
DrvAutoReq auto_req;

void ReadThread()
{
	
    int s,i;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;
	char *ifname = can2_name;

	int setflag,getflag,ret =0;
	struct can_filter rfilter[2];  //过滤

	//创建套接字
	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
	}

	//指定can设备，指定接口索引，还有后面的套接字与can绑定，这样只能接受到指定can设备的数据
	
	strcpy(ifr.ifr_name, ifname);        //指认can设备
	ioctl(s, SIOCGIFINDEX, &ifr);        //指定接口索引

	addr.can_family  = AF_CAN;
	//addr.can_ifindex = ifr.ifr_ifindex; 
	addr.can_ifindex = 0;            //为了将套接字和所有的CAN接口绑定，接口索引必须是0

	//将套接字与can绑定	
	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
	}
	
   /*********************************************************************************/

	setflag = setflag|O_NONBLOCK;   //非阻塞
	ret = fcntl(s,F_SETFL,setflag);
	getflag = fcntl(s,F_GETFL,0);

    //通过错误掩码可以实现对错误帧的过滤
	can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT |CAN_ERR_BUSOFF;
	ret = setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
	if (ret!=0)
		printf("setsockopt fail\n");
	
	//定义接收规则
	/*
	rfilter[0].can_id = 0x366;
	rfilter[1].can_id = 0x365;
	rfilter[0].can_mask = CAN_SFF_MASK;
	rfilter[1].can_mask = CAN_SFF_MASK;

	  //valid bits in CAN ID for frame formats 
     //#define CAN_SFF_MASK 0x000007FFU    //标准帧格式 (SFF) 
     //#define CAN_EFF_MASK 0x1FFDisplayFFFFFU    //扩展帧格式 (EFF)
     //#define CAN_ERR_MASK 0x1FFFFFFFU    //忽略EFF, RTR, ERR标志 
    

	//设置过滤规则
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    */

	printf("using %s to read\n", ifname);

	

    while(1) {
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if (nbytes == sizeof(struct can_frame)) {
		   	if (frame.can_id & CAN_ERR_FLAG)
				printf("error frame\n");
		   	else {
				   /*
			   printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
			   for (i =0;i<frame.can_dlc;i++) {
			  	  printf("data[%d]=0x%x \n",i,frame.data[i]);
				}
				*/
				switch(frame.can_id) {
					case EPS_1_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						eps.EPS_1_decode(frame);
						break;
					case EPS_11_id:
					    printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						eps.EPS_11_decode(frame);
						break;
					case EPB_1_id:
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						eps.EPB_decode(frame);
						break;
					case BCS_1_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						bcs.BCS_1_decode(frame);
						break;
					case BCS_2_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						bcs.BCS_2_decode(frame);
						break;
					case BCS_3_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						bcs.BCS_3_decode(frame);
						break;
					case BCS_5_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						bcs.BCS_5_decode(frame);
						break;
					case BCS_6_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						bcs.BCS_6_decode(frame);
						break;
					case BCS_7_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						bcs.BCS_7_decode(frame);
						break;
					case BCS_8_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						bcs.BCS_8_decode(frame);
						break;
					case BCS_9_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						bcs.BCS_9_decode(frame);
						break;
					case BCS_10_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						bcs.BCS_10_decode(frame);
						break;
					case VCU_2_P_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						vcu.VCU_2_P_decode(frame);
						break;
					case VCU_7_P_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						vcu.VCU_7_P_decode(frame);
						break;
					case VCU_9_P_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						vcu.VCU_9_P_decode(frame);
						break;
					case VCU_18_P_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						vcu.VCU_18_P_decode(frame);
						break;
					case VCU_19_P_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						vcu.VCU_19_P_decode(frame);
						break;
					case VCU_20_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						vcu.VCU_20_decode(frame);
						break;
					case VCU_21_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						vcu.VCU_21_decode(frame);
						break;
					case VCU_EMS_1_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						vcu.VCU_EMS_1_decode(frame);
						break;
					case VCU_EMS_6_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						vcu.VCU_EMS_6_decode(frame);
						break;
					case GSM_1_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						elseid.GSM_1_decode(frame);
						break;
					case SAS_1_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=start0x%x \n",i,frame.data[i]);
							}
						elseid.SAS_1_decode(frame);
						break;
					case GW_BCM_2_P_id:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						elseid.GW_BCM_2_P_decode(frame);
						break;
					//本地回环
					case SCU_2:
						printf("\nID=0x%x DLC=%d \n",frame.can_id,frame.can_dlc);
						for (i =0;i<frame.can_dlc;i++) {
			  	  				printf("data[%d]=0x%x \n",i,frame.data[i]);
							}
						steer_control.steer_loop_decode(frame);
						break;
					default:
						break;
				}
			}
		 }
		 
	}
}


//订阅遥控器的消息Display_control
void remote_msg(const remote_car::car_data& msg) {

	steer_control.angle_input = msg.steering_wheel;
	auto_req.DrvAuto_flag  = msg.Enter_steering_control;

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_node");
  	ros::NodeHandle node;

	clock_t start;
	start = clock();
	//opencv显示
	//opencv显示
	cv::Mat Display(Size(3200,1600), CV_8UC3, Scalar(255,255,255)); 
	namedWindow("display",0);


  	pthread_t thread_id;
  	typedef void* (*FUNC)(void*);
  	FUNC callback = (FUNC)&ReadThread;
  	int ret = pthread_create(&thread_id,NULL,callback,NULL);
  	if(ret!=0) 
  		return 1;
 
	int s;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;

	char *ifname = can2_name;
 
	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Error while opening socket");
	}

	//在默认情况下，本地回环功能是开启的，可以使用下面的方法关闭回环/开启功能：
	// int loopback = 0; // 0表示关闭, 1表示开启(默认)
    // setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex; 

	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) 
	{
		perror("Error in socket bind");
	}

	ros::Subscriber remote_subscriber = node.subscribe("car_remote_data", 20, &remote_msg);

	
	ros::Rate loop_rate(20);
    while(ros::ok()){


		// 接收到遥控器进入EPS自动模式的请求
		if(auto_req.DrvAuto_flag == 1) {

			steer_control.in_steer_auto(eps);

			//进入EPS自动模式后发送接收到50的方向盘遥控指令
			if(eps.EPS_ControlMode == 2 && eps.EPS_FailureSt == false) {
				steer_control.steer_control_code();
				}

			nbytes = write(s, &steer_control.frame, sizeof(struct can_frame));
	  		printf("\nID=0x%x DLC=%d \n",steer_control.frame.can_id,frame.can_dlc);
			for (int i =0;i<steer_control.frame.can_dlc;i++) {
				printf("data[%d]=0x%x \n",i,steer_control.frame.data[i]);
				}
	  		if(nbytes != sizeof(steer_control.frame)) //如果nbytes不等于帧长度，就说明发送失败
				printf("Error\n!");
		}

		else {
			
			if(eps.EPS_ControlMode == 2 && eps.EPS_FailureSt == false){

				steer_control.esc_steer_auto(eps);
				nbytes = write(s, &steer_control.frame, sizeof(struct can_frame));
	  			printf("\nID=0x%x DLC=%d \n",steer_control.frame.can_id,frame.can_dlc);
				for (int i =0;i<steer_control.frame.can_dlc;i++) {
					printf("data[%d]=0x%x \n",i,steer_control.frame.data[i]);
					}
	  			if(nbytes != sizeof(steer_control.frame)) //如果nbytes不等于帧长度，就说明发送失败
					printf("Error\n!");
			}
		}

		//大约0.2s刷新一次显示
		 if( (clock() - start) >= 200000) {
			 start = clock();
			 Display.setTo(Scalar(255, 255, 255));
			 eps.EPS_display(Display);
			 bcs.BCS_display(Display);
			 vcu.VCU_display(Display);
			 elseid.ELSE_display(Display);
			 steer_control.steer_control_dis(eps, Display);

             imshow("display", Display);
	         waitKey(1);
		 }
		
	  	ros::spinOnce();
	  	loop_rate.sleep();
	}

  return 0;
}


