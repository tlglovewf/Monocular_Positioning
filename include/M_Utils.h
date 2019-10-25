//
//  M_Utils.h
//
//  Created by TuLigen on 2019/6/4.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef _M_UTILS_H_H_
#define _M_UTILS_H_H_

#include "M_Types.h"
#include "M_CoorTransform.h"
#include <ctime>
#define GPS_LEAP_TIME     18.0   //GPS 闰秒
//工具类
class M_Untils
{
public:
    
    /****************************************************
     **********************时间处理***********************
     ****************************************************/
    
    /* 获取拉格朗日插值
     *
     */
    static double inline GetLxValue(double x,double x1,double x2, double y1, double y2)
    {
        double dx = x1 - x2;
        
        return y1 * (x - x2) / dx + y2 * (x - x1) / -dx;
    }
    
    /* "hhmmsszzzzzz"   ->    天秒
     * "031953179985"
     */
    static inline double HMS2DaySec(const std::string &str)
    {
        int h;
        int m;
        int s;
        int z;
        sscanf(str.c_str(),"%2d%2d%2d%6d", &h,&m,&s,&z);
        
        double daysec = h * 3600 + m * 60 + s + z / 1.0e6 + GPS_LEAP_TIME;
        
        return daysec;
    }
    
    /* 时间转换
     * 根据解算后图片名 转天秒
     */
    static inline double GetDayTimeFromPicName(const std::string &pic)
    {
        size_t n = pic.find_last_of('/');
        if( n == string::npos)
        {
            return  HMS2DaySec(pic.substr(9,12));
        }
        else
        {
            string date = pic.substr(++n).substr(9,12);
            return HMS2DaySec(date);
        }
    }
    
    /* 周秒 -> 天秒
     *
     */
    static inline double WeekSec2DaySec(double wktime)
    {
        const double daysec = 86400;
        
        int iday = wktime / daysec;
        
        double dtime = wktime - iday * daysec;
        
        return dtime;
    }
    
    /* 天秒 -> 时分秒
     *
     */
    static std::string DaySec2HMS(double dtime)
    {
        dtime -= GPS_LEAP_TIME;
        
        int zz = (dtime - (int)dtime)*1e6;
        
        int hh = (int)dtime / 3600;
        
        int _m = (int)dtime - hh * 3600;
        
        int mm = _m / 60;
        
        int ss = _m - mm * 60;
        
        std::string s;
        std::stringstream str;
        if(hh < 10)
            str << 0;
        str << hh;
        str << mm;
        str << ss;
        str << zz;
        
        str >> s;
        
        return s;
    }

    /*
    *  周秒 -> 天秒
    */
   static double Wktime2Daytime(double wktime)
{
    //天秒
    static const int daysec = 3600 * 24;
    double msec = wktime - static_cast<int>(wktime);
    int sttime = (static_cast<int>(wktime) % daysec);
    return (sttime + msec);
}

    
    /* 从位姿获取R和t
     *
     */
    static void GetRtFromPose(const PoseData &predata,
                              const PoseData &curdata,
                              const Mat cam2imuR,
                              const Mat cam2imuT,
                              Mat &R, Mat &t)
    {
        const BLHCoordinate &blht1  = predata.pos;
        const BLHCoordinate &blht2  = curdata.pos;
        
        //计算imu到enu 转换矩阵
        cv::Mat Rimu2Enu1 = M_CoorTrans::IMU_to_ENU(-predata._yaw, predata._pitch, predata._roll);
        cv::Mat Rimu2Enu2 = M_CoorTrans::IMU_to_ENU(-curdata._yaw, curdata._pitch, curdata._roll);
        
        //计算xyz转到enu 转换矩阵
        cv::Mat XYZ2Enu1 = M_CoorTrans::XYZ_to_ENU(blht1.latitude, blht1.longitude);
        cv::Mat XYZ2Enu2 = M_CoorTrans::XYZ_to_ENU(blht2.latitude, blht2.longitude);
        
        //imu到 xyz转换矩阵
        cv::Mat Rimu2xyzt1 = XYZ2Enu1.t() * Rimu2Enu1;
        cv::Mat Rimu2xyzt2 = XYZ2Enu2.t() * Rimu2Enu2;
        
        
        Point3d xyzt1;
        Point3d xyzt2;
        //获取xyz坐标
        xyzt1 = M_CoorTrans::BLH_to_XYZ(blht1);
        xyzt2 = M_CoorTrans::BLH_to_XYZ(blht2);
        cv::Mat pt1 = (cv::Mat_<double>(3, 1) << xyzt1.x, xyzt1.y, xyzt1.z);
        cv::Mat pt2 = (cv::Mat_<double>(3, 1) << xyzt2.x, xyzt2.y, xyzt2.z);
        
        //相对旋转矩阵
        
        R = cam2imuR.t() * Rimu2xyzt2.t() * Rimu2xyzt1 * cam2imuR;//R
    
        //计算cur相机在xyz坐标系中坐标
        cv::Mat curCamPos = Rimu2xyzt2 * cam2imuT + pt2;
        //计算以pre为原点建立的imu坐标系,pt2的位置
        cv::Mat imut1Pcam = Rimu2xyzt1.t() * curCamPos - Rimu2xyzt1.t() * pt1;
        //计算以pre为原点建立的imu坐标系,pt2相机的位置
        cv::Mat camt1Pcam = cam2imuR.t() * imut1Pcam - cam2imuR.t() * cam2imuT;
        
        //以cam2的位置 反推t  这里r * cam2 只是计算方向
        t = -R * camt1Pcam;//t
    }
    
    
    /* 通过帧间R、t推算绝对坐标
     *
     */
    static void CalcPoseFromRT(const PoseData &origin,
                               const Mat &R, const Mat &t,
                               const Mat &cam2imuR,
                               const Mat &cam2imuT,
                               BLHCoordinate &blh,
                               const PoseData &realdst={0})
    {
        const BLHCoordinate &ogngps = origin.pos;
        
        const Point3d xyz = M_CoorTrans::BLH_to_XYZ(ogngps);
        
        const Mat m_xyz = (Mat_<double>(3,1) << xyz.x,xyz.y,xyz.z);
        
        Mat imu2enu = M_CoorTrans::IMU_to_ENU(-origin._yaw, origin._pitch, origin._roll);
        
        const Mat xyz2enu = M_CoorTrans::XYZ_to_ENU(ogngps.latitude, ogngps.longitude);
        
        //imu坐标系->xyz坐标系旋转矩阵
        Mat imu2xyz = xyz2enu.inv() * imu2enu ;
        //计算pre相机坐标系 cur相机位置
        Mat dstcampt = t;//-R.inv() * t;
        //相机坐标系->imu坐标系
        Mat dstimupt = cam2imuR * dstcampt + cam2imuT;
        //imu坐标系->xyz坐标系
        Mat dstxyzpt = imu2xyz * dstimupt + m_xyz;
        //cam位置转到imu位置
        dstxyzpt = dstxyzpt - imu2xyz * cam2imuT;
        
        Point3d  dstxyz(dstxyzpt.at<double>(0,0),
                        dstxyzpt.at<double>(1,0),
                        dstxyzpt.at<double>(2,0));
        
        blh = M_CoorTrans::XYZ_to_BLH(dstxyz);
        
        if( realdst._t > 0)
        {
            Point3d orngauss = M_CoorTrans::BLH_to_GaussPrj(realdst.pos);
            Point3d dstgauss = M_CoorTrans::BLH_to_GaussPrj(blh);
            
            cout << "dif: "
                 << orngauss.x - dstgauss.x << " "
                 << orngauss.y - dstgauss.y << " "
                 << orngauss.z - dstgauss.z << endl;
        }
    }
    
    /****************************************************
     **********************视觉相关***********************
     ****************************************************/
    
    //获取反对称矩阵
    static inline cv::Mat antisymMat(const cv::Mat &t)
    {
        cv::Mat t_x = (cv::Mat_<double>(3, 3) <<
                       0, -t.at<double>(2, 0), t.at<double>(1, 0),
                       t.at<double>(2, 0), 0, -t.at<double>(0, 0),
                       -t.at<double>(1, 0), t.at<double>(0, 0), 0);
        return t_x;
    }
    //计算本质矩阵
    static inline cv::Mat ComputeEssentialMat(const cv::Mat &R, const cv::Mat &T)
    {
        return antisymMat(T)*R;
    }
    //计算基础矩阵
    static inline cv::Mat ComputeFundamentalMat(const cv::Mat &E12, const cv::Mat &K1, const cv::Mat &K2)
    {
        return (K2.inv()).t()*E12*K1.inv();
    }
    //根据基础矩阵 创建 ax + by + c = 0 极线
    static void CreateEpiline(const cv::Mat& F, cv::Point2f pt, double& a, double& b, double& c)
    {
        std::vector <cv::Point2f>  selPoints1;
        selPoints1.push_back(pt);
        std::vector<cv::Vec3f> epline;

        cv::computeCorrespondEpilines(cv::Mat(selPoints1), 1, F, epline);

        a = epline[0][0];
        b = epline[0][1];
        c = epline[0][2];
    }
    
    //像素坐标系->图像坐标系
    static inline cv::Point2d Pixel2Cam(const cv::Point2d &p, const cv::Mat &K)
    {
        return cv::Point2d
        (
         (p.x-K.at<double>(0,2))/K.at<double>(0,0),
         (p.y-K.at<double>(1,2))/K.at<double>(1,1)
         );
    }
    
    //y = (-a*x - c) / b   计算y值
    static inline double ComputeY(float x, float a, float b, float c)
    {
        assert( fabs(b) > 1e-6);
        return (-c - a * x) / b;
    }
    
    
    /*
     * 叉乘
     */
    inline static float  segmentCross
    (
     const Point2f&    v,
     const Point2f&    vStart,
     const Point2f&    vEnd
     )
    {
        return (vStart.x-v.x)*(vEnd.y-v.y) - (vEnd.x-v.x)*(vStart.y-v.y);
    }
    
#define YD_EPS_REAL32 1e-6
    /**
     * 判断线段与线段是否相交(跨立试验）
     * @param vStart1,vEnd1 线段1起终点
     * @param vStart2,vEnd2 线段2起终点
     * @return 是否相交
     */
    static bool  IsIntersect(const Point2f&    vStart1,const Point2f&    vEnd1,
                             const Point2f&    vStart2,const Point2f&    vEnd2)
    {
        float leftS, leftE;
        leftS = segmentCross(vStart1, vStart2, vEnd2);
        leftE = segmentCross(vEnd1, vStart2, vEnd2);
        if ( leftS * leftE > YD_EPS_REAL32 )
        {
            return false;       // vStart1, vEnd1在另一条直线的同侧
        }
        
        leftS = segmentCross(vStart2, vStart1, vEnd1);
        leftE = segmentCross(vEnd2, vStart1, vEnd1);
        if ( leftS * leftE > YD_EPS_REAL32 )
        {
            return false;       // vStart2, vEnd2在另一条直线的同侧
        }
        
        return true;
    }
    
    /**
     * 判断直线是否与矩形相交
     * @param bg,ed 线段起终点
     * @param rect  矩形
     */
    static bool IsIntersect(const Point2f &bg,const Point2f &ed, const Rect2f &rect)
    {
        if(rect.contains(bg) || rect.contains(ed))
        {//先判断直线起终点是否在矩形内
            return true;
        }
        else
        {//再判断直线是否与矩形对角线相交
            return IsIntersect(bg, ed, Point2f(rect.x,rect.y),
                               Point2f(rect.x+rect.width,rect.y+rect.height))||
            IsIntersect(bg, ed, Point2f(rect.x + rect.width,rect.y),
                        Point2f(rect.x,rect.y+rect.height));
        }
    }
    

	/**
	* 获取文件名No
	* @param  name  名
	* @return rect  序号
	*/
	static inline std::string GetNameNo(const std::string &name)
	{
		int index = name.find_last_of('-');

		std::string tmp = name.substr(++index);

		tmp.erase(tmp.find_last_of('.'), 4);

		index = tmp.find_first_not_of('0');

		return tmp.substr(index);
	}
};

//及时
class Time_Interval
{
public:
    /*  开始
     */
    inline void   start()
    {
        _t = clock();
    }
    /*  结束
     */
    inline float  end()
    {
        return (clock() - _t)/(float)CLOCKS_PER_SEC;
    }
	/* 输出
	 */
	inline void prompt(const std::string &str)
	{
		std::cout << str.c_str() << end() << "s" << std::endl;
	}
protected:
    time_t _t;
};

#endif /* Monocular_Utils_h */
