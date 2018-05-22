

#include<iostream>
#include "JY901.h"
#include "rs232.h"
#include <ros/ros.h>
#include "odomIMU_msgs/odomIMU.h"
using namespace std;

ros::Publisher odomIMU_pub;
enum dataType_ {

    STIME = 0,
    SACC,
    SGYRO,
    SANGLE,
    SMAG,
    SDSTATUS,
    SPRESS,
    SLONLAT,
    SGPSV,
    SQ,
    ENCODE0CNT,
    ENCODE1CNT
};

#pragma pack(4)

typedef struct IMU_{
    struct STime stcTime; //0
    struct SAcc stcAcc; // 1
    struct SGyro stcGyro; // 2
    struct SAngle stcAngle; // 3
    struct SMag stcMag; // 4
    struct SDStatus stcDStatus; // 5
    struct SPress stcPress; // 6
    struct SLonLat stcLonLat; // 7
    struct SGPSV stcGPSV; // 8
    struct SQ stcQ; // 9

}imuData;

typedef struct SensorData_{
    char head[4];
    imuData imu;
    int Encode0Cnt; // 10
    int Encode1Cnt; // 11

}SensorData;
#pragma pack()

static unsigned char ucRxBuffer[250];
static unsigned char ucRxCnt = 0;

int parseSensorData(char Byte){

    ucRxBuffer[ucRxCnt++] = Byte;
    if(ucRxCnt < 4){
        return 0;
    }

    if(ucRxBuffer[0] != 0x50 &&
    ucRxBuffer[1] != 0x51 &&
    ucRxBuffer[2] != 0x52 &&
    ucRxBuffer[3] != 0x53){
        ucRxCnt --;
        memmove(ucRxBuffer , &ucRxBuffer[1] , ucRxCnt);
        return 0;
    }

    if(ucRxCnt < sizeof(SensorData)){
        return 0;
    }
    else{
        ucRxCnt = 0;
    }

    SensorData *p = (SensorData *)ucRxBuffer;

    odomIMU_msgs::odomIMU msg;
    //odomIMU_msgs::odomIMU msg;
	msg.header.frame_id = "odomIMU_frame_info";
	msg.header.stamp = ros::Time::now();

    //            imu_data.angular_velocity.x = (float)jy901_frame_sst.stcGyro.w[0]/32768*2000;
    //            imu_data.angular_velocity.y = (float)jy901_frame_sst.stcGyro.w[1]/32768*2000;
    //            imu_data.angular_velocity.z = (float)jy901_frame_sst.stcGyro.w[2]/32768*2000;

    //            imu_data.linear_acceleration.x = (float)jy901_frame_sst.stcAcc.a[0]/32768*16;
    //            imu_data.linear_acceleration.y = (float)jy901_frame_sst.stcAcc.a[1]/32768*16;
    //            imu_data.linear_acceleration.z = (float)jy901_frame_sst.stcAcc.a[2]/32768*16;
    //            roll = (float)jy901_frame_sst.stcAngle.Angle[0]/32768*180;
    //            pitch = (float)jy901_frame_sst.stcAngle.Angle[1]/32768*180;
    //            yaw = (float)jy901_frame_sst.stcAngle.Angle[2]/32768*180;
    msg.gyro.resize(3);
	msg.gyro[0] = (float)((short)(p->imu.stcGyro.w[0]))/32768*2000;
	msg.gyro[1] = (float)((short)(p->imu.stcGyro.w[1]))/32768*2000;
	msg.gyro[2] = (float)((short)(p->imu.stcGyro.w[2]))/32768*2000;	

    msg.acc.resize(3);
	msg.acc[0] = (float)((short)(p->imu.stcAcc.a[0]))/32768*16;
	msg.acc[1] = (float)((short)(p->imu.stcAcc.a[1]))/32768*16;
	msg.acc[2] = (float)((short)(p->imu.stcAcc.a[2]))/32768*16;

    msg.angle.resize(3);
	msg.angle[0] = (float)((short)(p->imu.stcAngle.Angle[0]))/32768*180;
	msg.angle[1] = (float)((short)(p->imu.stcAngle.Angle[1]))/32768*180;
	msg.angle[2] = (float)((short)(p->imu.stcAngle.Angle[2]))/32768*180;
	
    int encode0 = p->Encode0Cnt;
    int encode1 = p->Encode1Cnt;
	msg.encode0Cnt = encode0;//p->Encode0Cnt;
	msg.encode1Cnt = encode1;//p->Encode1Cnt;

    // encode range(-6044,8660) ~{180degree,0degree}
	odomIMU_pub.publish(msg);

    //printf("stcTime:Y%04d M%04d D%04d h%04d m%04d s%04d ms%04d \n" , p->imu.stcTime.ucYear , p->imu.stcTime.ucMonth,
    //    p->imu.stcTime.ucDay , p->imu.stcTime.ucHour , p->imu.stcTime.ucMinute, p->imu.stcTime.ucSecond,
    //    p->imu.stcTime.usMiliSecond);

    //printf("stcAcc:a %02x %d %d %d T%02x\n" , p->imu.stcAcc.a[0], (short)(p->imu.stcAcc.a[0]) , (short)(p->imu.stcAcc.a[1]), (short)(p->imu.stcAcc.a[2]), p->imu.stcAcc.T);
    //printf("stcGyro:w%d %d %d T%02x\n" , p->imu.stcGyro.w[0], p->imu.stcGyro.w[1] ,p->imu.stcGyro.w[2] , p->imu.stcGyro.T);
    //printf("stcAngle:Angle%02x T%02x\n" , p->imu.stcAngle.Angle , p->imu.stcAngle.T);
    //printf("stcMag:h%02x T%02x\n" , p->imu.stcMag.h , p->imu.stcMag.T);
    //printf("stcDStatus:a%02x T%02x\n" , p->imu.stcDStatus.sDStatus);
    //printf("stcPress:a%02x T%02x\n" , p->imu.stcPress.lAltitude , p->imu.stcPress.lPressure);
    //printf("stcLonLat:a%02x T%02x\n" , p->imu.stcLonLat.lLat , p->imu.stcLonLat.lLon);
    //printf("stcGPSV:%02x %02x %02x\n" , p->imu.stcGPSV.lGPSVelocity , p->imu.stcGPSV.sGPSHeight, p->imu.stcGPSV.sGPSYaw);
    //printf("stcQ:%02x\n" , p->imu.stcQ.q);
    //printf("Encode0Cnt:%02x %d\n" , p->Encode0Cnt, (int)(p->Encode0Cnt));
    //printf("Encode1Cnt:%02x %d\n" , p->Encode1Cnt, (int)(p->Encode0Cnt));
}

int main(int argc, char **argv){

    ros::init(argc, argv, "mapBuilder");
    ros::start();
	
	ros::NodeHandle nh;
	odomIMU_pub = nh.advertise<odomIMU_msgs::odomIMU>("odomIMU_frame" , 100);
    int i,n,
    cport_nr = 16, //
    bdrate = 921600;
    
    unsigned char buf[4096];

    char mode[] = {'8' , 'N' , '1' , 0};

    short key = 0xFFB7;
    short key1 = 0x0494;
    short key2 = 0xF905;
    std::cout << key << " " << key1 << " " << key2 <<  std::endl;
        std::cout << (float)key/32768*16 << " " << (float)key1/32768*16 << " " << (float)key2/32768*16 <<  std::endl;
    float ss1 = (float)key/32768*16;
    float ss2 = (float)key1/32768*16;
    float ss3 = (float)key2/32768*16;
    std::cout << ss1 * ss1 + ss2 * ss2 + ss3 * ss3 << std::endl;
    if(RS232_OpenComport(cport_nr , bdrate , mode)){
        printf("Can not open comport\n");
        return 0;
    } 

    while(ros::ok()){
        
        n = RS232_PollComport(cport_nr , buf , 4095);
        if(n>0){
            buf[n] = 0;
            for(int i=0;i<n;i++){
                parseSensorData(buf[i]);
            }
        }

        ros::spinOnce();
        //usleep(100000);
    }

    ros::shutdown();

    return 0;
}
