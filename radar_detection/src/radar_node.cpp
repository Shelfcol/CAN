#include <iostream>
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "ICANCmd.h"

using namespace std;

#define height 0
DWORD dwDeviceHandle;
CAN_InitConfig config;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "radar");
	ros::NodeHandle nh;
	//ros::Publisher pub_1 = nh.advertise<perception_sensor_msgs::ObjectList>("/radar", 2);
	ros::Publisher pub_2 = nh.advertise<sensor_msgs::PointCloud2>("/radar_projection", 2);
	
	if(dwDeviceHandle = CAN_DeviceOpen(ACUSB_132B, 0, 0) == 1)
	{
		ROS_INFO_STREAM(" >>open device success!");
	}
	else
	{
		ROS_ERROR_STREAM(" >>open device error!");
		return 0;
        exit(1);
	}

	CAN_InitConfig config;
   	config.dwAccCode = 0;
   	config.dwAccMask = 0xffffffff;
   	config.nFilter  = 0;       // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
   	config.bMode    = 0;             // 工作模式(0表示正常模式,1表示只听模式)
   	config.nBtrType = 1;      // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
   	config.dwBtr[0] = 0x00;   // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
   	config.dwBtr[1] = 0x1c;   // BTR1
   	config.dwBtr[2] = 0;
   	config.dwBtr[3] = 0;
    
	if( CAN_ChannelStart(dwDeviceHandle, 0, &config) != CAN_RESULT_OK )
	{
		ROS_ERROR_STREAM(" >>Init CAN0 error!");

	    return 0;
	}

	if( CAN_ChannelStart(dwDeviceHandle, 1, &config) != CAN_RESULT_OK )
	{
		ROS_ERROR_STREAM(" >>Init CAN1 error!");

	    return 0;
	}
  
	int reclen = 0;
	CAN_DataFrame rec[3000]; //buffer
	int CANInd = 0; //CAN1=0, CAN2=1

	vector<float> xx_list;
	vector<float> yy_list;
	vector<float> vx_list;
	vector<float> vy_list;
	vector<float> confidence_list;
	vector<int> obj_id_list;





	vector<CAN_DataFrame> frame_obj;

	while(ros::ok())
	{
		if((reclen=CAN_ChannelReceive(dwDeviceHandle, 0, rec, 3000, 100)) > 0) //调用接收函数,得到数据
		{

			unsigned int num_of_obj;
			float xx, yy, vx, vy, prob;
			int ID;

			for(int i=0; i<reclen; i++)
			{
				// push back the objects of previous frame????????????????????????????
				if(rec[i].uID != 0x60A)
				{
					frame_obj.push_back(rec[i]);//?
				}

				// release the objects of previous frame
				if(rec[i].uID == 0x60A)
                {
                
                    num_of_obj = rec[i].arryData[0];//?
                    cout << "number of objects = " << num_of_obj << endl;

					for(int j = 0; j < frame_obj.size(); j++)
					{
						if(frame_obj[j].uID == 0x60B)
						{
							// ID
							ID = (frame_obj[j].arryData[0]>>0);
							// distance x
							xx = ((frame_obj[j].arryData[1]<<5)|(frame_obj[j].arryData[2]>>3))*0.2 - 500;
							// distance y
							yy = (((frame_obj[j].arryData[2]&0x07)<<8)|(frame_obj[j].arryData[3]>>0))*0.2 - 204.6;
							// velocity x
							vx = ((frame_obj[j].arryData[4]<<2)|(frame_obj[j].arryData[5]>>6))*0.25 - 128;
							// velocity y
							vy = (((frame_obj[j].arryData[5]&0x3F)<<3)|(frame_obj[j].arryData[6]>>5))*0.25 - 64;

							cout << xx << '\t' << yy << '\t' << vx << '\t' << vy << endl; 
							
							obj_id_list.push_back(ID);
							xx_list.push_back(xx); 
							yy_list.push_back(yy); 
							vx_list.push_back(vx); 
							vy_list.push_back(vy); 
						}
						if(frame_obj[j].uID == 0x60C)
						{
							// existance probability
							prob = frame_obj[j].arryData[6]>>5;
							confidence_list.push_back(prob);
						}
					}
					
					// pub for projection
					pcl::PointCloud<pcl::PointXYZ> radar_point;
					pcl::PointXYZ point;

					for(int k = 0; k < xx_list.size(); k++)
					{

						// pub for projection
						point.x = xx_list[k];
						point.y = yy_list[k];
						point.z = height;
						radar_point.push_back(point);
					}

					// pub for projection
					sensor_msgs::PointCloud2 msg_radar;
					pcl::toROSMsg(radar_point, msg_radar);
					msg_radar.header.frame_id = "radar";
					msg_radar.header.stamp = ros::Time::now();
					pub_2.publish(msg_radar);
					radar_point.clear();
					xx_list.clear();
					yy_list.clear();
					vx_list.clear();
					vy_list.clear();
					confidence_list.clear();
					obj_id_list.clear();
					frame_obj.clear();
                }
            }
		}
	}
	 CAN_DeviceClose(dwDeviceHandle);
	return 0;
}
