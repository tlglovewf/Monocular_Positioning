//
//  Ｍ_Types.h
//
//  Created by TuLigen on 2019/6/4.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef _M_TYPES_H_H_
#define _M_TYPES_H_H_

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <memory>

#ifdef _MONOCULAR_DLL_

#define _POSITIONING_DLL_ _declspec(dllexport)

#else

#define _POSITIONING_DLL_ _declspec(dllimport)

#endif

using namespace std;
using namespace cv;


#define WRONGDATA      -1000  //错误值
#define OUTPUTRESULT    1

//经纬度
struct BLHCoordinate
{
    double latitude;
    
    double longitude;
    
    double altitude;
    
    BLHCoordinate(double lat,double lon, double alt):latitude(lat),longitude(lon),altitude(alt){}
    
	BLHCoordinate() :altitude(WRONGDATA){}
    
    //有效性
    static inline bool isValid(const BLHCoordinate &blh)
    {
		return blh.altitude > WRONGDATA;
    }
};

//相机结构体
struct Camera
{
    Mat K;           //相机内参
    Mat RCam2Imu;    //相机->IMU坐标系 旋转矩阵
    Mat TCam2Imu;    //相机->IMU坐标系 平移矩阵
    //add more
};

//姿态结构体
struct PoseData {
    double _t;          //时间
    
    BLHCoordinate pos;  //位置
    
    double _pitch;      //俯仰角
    double _roll;       //翻滚角
    double _yaw;        //航偏角
};

//imu 六轴数据
struct ImuRawData
{
    double _t;
    double _gyro_x;
    double _gyro_y;
    double _gyro_z;

    double _acc_x;
    double _acc_y;
    double _acc_z;  

    ImuRawData(double t, double gx,double gy, double gz,
                          double ax,double ay, double az):_t(t),_gyro_x(gx),_gyro_y(gy),_gyro_z(gz),
                                                          _acc_x(ax),_acc_y(ay),_acc_z(az)
                          {

                          }
    ImuRawData()
    {
        memset(this,0,sizeof(*this));
    }
};

typedef std::vector<PoseData>   PoseVector;
typedef PoseVector::iterator    PoseVIter;

typedef std::vector<ImuRawData> IMURawVector;
typedef IMURawVector::iterator  IMURawVIter;


struct Target {
  
    int           _type;     //类型
    
    Rect2f        _box;      //目标包围框
    
    BLHCoordinate _pos;      //无值
    
    //中心点
    inline Point2f center()const
    {
        return (_box.tl() + _box.br()) / 2;
    }
    
    //有效性
    static inline bool isValid(const Target &target)
    {
		return target._type > WRONGDATA;
    }
	Target() :_type(WRONGDATA){}
};

typedef std::vector<Target>     TargetVector;
typedef TargetVector::iterator  TargetVIter;

//帧数据
struct FrameData {
    
    PoseData        _pose;     //姿态
    
	std::string     _name;     //路径

    Mat             _img;      //图像
    
    TargetVector    _targets;  //目标集
    
    //插入
    void push_back(const Target &target)
    {
        _targets.emplace_back(target);
    }
};

//结果
struct Result
{
    int           _type;        //类型
    
    BLHCoordinate _pos;         //坐标
    
};


typedef std::vector<Result>   ResultVector;

#endif /* Monocular_Types_h */
