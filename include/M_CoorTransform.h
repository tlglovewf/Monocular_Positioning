//
//  CoorTransform.hpp
//  
//
//  Created by TuLigen on 2019/6/10.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef _M_COORTRANSFORM_H_H_
#define _M_COORTRANSFORM_H_H_

#include <opencv2/core.hpp>
#include "M_Types.h"
using namespace cv;
#define D2R(X)   (X) * 0.01745329251994329547437168059786927187815308570862
#define R2D(X)   (X) / 0.01745329251994329547437168059786927187815308570862

const double PI64 = 3.1415926535897932384626433832795028841971693993751;//Pi
struct Datum
{
    double r_max;
    double r_min;
    double e2;
};
extern Datum WGS84Datum;

//坐标转换类(静态类)
class M_CoorTrans
{
public:
    //大地坐标->地心地固空间直角坐标系
    static Point3d BLH_to_XYZ(const BLHCoordinate &blh, Datum dt = WGS84Datum)
    {
        double a = dt.r_max;
        double e = dt.e2;//sqrt(a * a - b * b) / a;
        double r_lat = D2R(blh.latitude);
        double r_lon = D2R(blh.longitude);
        double N = a / sqrt(1 - e  * sin(r_lat) * sin(r_lat));
        double WGS84_X = (N + blh.altitude) * cos(r_lat) * cos(r_lon);
        double WGS84_Y = (N + blh.altitude) * cos(r_lat) * sin(r_lon);
        double WGS84_Z = (N * (1 - e) + blh.altitude) * sin(r_lat);
        
        return{ WGS84_X, WGS84_Y, WGS84_Z };
    }
    //地心地固坐标系->大地坐标系
    static BLHCoordinate XYZ_to_BLH(const Point3d &pt, Datum dt = WGS84Datum)
    {
        double f, f1, f2;
        double p, zw, nnq;
        double b, l, h;
        
        double a = dt.r_max;
        double eq = dt.e2;
        f = PI64 * 50 / 180;
        double x, y, z;
        x = pt.x;
        y = pt.y;
        z = pt.z;
        
        double ddp = x * x + y * y;
        
        p = z / sqrt(ddp);
        do
        {
            zw = a / sqrt(1 - eq * sin(f) * sin(f));
            nnq = 1 - eq * zw / (sqrt(ddp) / cos(f));
            f1 = atan(p / nnq);
            f2 = f;
            f = f1;
        } while (!(abs(f2 - f1) < 10E-10));
        b = R2D(f);
        l = R2D(atan(y / x));
        if (l < 0)
            l += 180.0;
        h = sqrt(ddp) / cos(f1) - a / sqrt(1 - eq * sin(f1) * sin(f1));
        return{ b, l, h };
    }
    
    //计算 xyz坐标系转enu坐标系 旋转矩阵
    static cv::Mat XYZ_to_ENU(const double& B, const double& L)
    {
        double r_L = D2R(L);
        double r_B = D2R(B);
        
        cv::Mat XYZ2ENU = (cv::Mat_<double>(3, 3) <<
                           -sin(r_L)           , cos(r_L)            , 0       ,
                           -sin(r_B) * cos(r_L), -sin(r_L) * sin(r_B), cos(r_B),
                           cos(r_L) * cos(r_B),  cos(r_B) * sin(r_L), sin(r_B));
        return XYZ2ENU;
    }
    
    //计算 imu坐标系 转enu坐标系
   static cv::Mat IMU_to_ENU(const double& yaw,const double& pitch,const double& roll)
    {
        
        double r_pitch  = D2R(pitch);
        double r_roll   = D2R(roll);
        double r_yaw    = D2R(yaw);
        
        //绕y轴
        cv::Mat rollR = (cv::Mat_<double>(3, 3)
                         << cos(r_roll), 0, sin(r_roll),
                         0, 1, 0,
                         -sin(r_roll), 0, cos(r_roll));
        
        //绕x轴
        cv::Mat pitchR = (cv::Mat_<double>(3, 3) <<
                          1, 0, 0,
                          0, cos(r_pitch), -sin(r_pitch),
                          0, sin(r_pitch), cos(r_pitch));
        
        //绕z轴
        cv::Mat yawR = (cv::Mat_<double>(3, 3) <<
                        cos(r_yaw), -sin(r_yaw), 0,
                        sin(r_yaw), cos(r_yaw), 0,
                        0, 0, 1);
        return yawR * pitchR * rollR;
    }

  
   //经纬度转高斯投影坐标（B 维度  L 精度   H 高程)
   static Point3d BLH_to_GaussPrj(const BLHCoordinate &BLH, Datum datum = WGS84Datum)//,double lon)
   {
	   int ProjNo, ZoneWide; ////带宽
	   double longitude0, X0, xval, yval;
	   double a, e2, ee, NN, T, C, A, M, b, l;//, h;
	   b = BLH.latitude;
	   l = BLH.longitude;
	   //    h = BLH.z;
	   ZoneWide = 3; //3度带宽
	   a = datum.r_max;
	   ProjNo = (int)((l - 1.5) / ZoneWide + 1);
	   longitude0 = ProjNo * ZoneWide; //中央经线

	   longitude0 = D2R(longitude0);
	   l = D2R(l); //经度转换为弧度
	   b = D2R(b); //纬度转换为弧度
	   e2 = datum.e2;
	   ee = e2 * (1.0 - e2);
	   NN = a / sqrt(1.0 - e2 * sin(b) * sin(b));
	   T = tan(b) * tan(b);
	   C = ee * cos(b) * cos(b);
	   A = (l - longitude0) * cos(b);

	   M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * b - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * sin(2 * b) + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * b) - (35 * e2 * e2 * e2 / 3072) * sin(6 * b));
	   xval = NN * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
	   yval = M + NN * tan(b) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A * A * A / 720);
	   X0 = 500000L;
	   xval = xval + X0;
	   return Point3d(yval, xval, BLH.altitude);
   }


   //高斯投影由大地平面坐标(Unit:Metres)反算经纬度(Unit:DD)
   static BLHCoordinate GaussPrj_to_BLH(Point3d XYZ, double lon, Datum datum = WGS84Datum)
   {
	   int ProjNo, ZoneWide; ////带宽
	   double l, b, longitude0, X0, xval, yval;
	   double e1, e2, a, ee, NN, T, C, M, D, R, u, fai;
	   a = datum.r_max; //54年北京坐标系参数
	   ZoneWide = 3; //3度带宽

	   ProjNo = (int)(XYZ.y / 1000000L); //查找带号
	   longitude0 = (int)((lon - 1.5) / ZoneWide + 1) * ZoneWide; //中央经线

	   longitude0 = D2R(longitude0); //中央经线
	   X0 = ProjNo * 1000000L + 500000L;
	   yval = XYZ.y - X0; //带内大地坐标
	   xval = XYZ.x;
	   e2 = datum.e2;
	   e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
	   ee = e2 / (1 - e2);
	   M = xval;
	   u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
	   fai = u + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * u) + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * u) + (151 * e1 * e1 * e1 / 96) * sin(6 * u) + (1097 * e1 * e1 * e1 * e1 / 512) * sin(8 * u);
	   C = ee * cos(fai) * cos(fai);
	   T = tan(fai) * tan(fai);
	   NN = a / sqrt(1.0 - e2 * sin(fai) * sin(fai));

	   R = a * (1 - e2) / sqrt((1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)));
	   D = yval / NN;
	   //计算经度(Longitude) 纬度(Latitude)
	   l = longitude0 + (D - (1 + 2 * T + C) * D * D * D / 6 + (5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D
		   * D * D * D * D / 120) / cos(fai);
	   b = fai - (NN * tan(fai) / R) * (D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24
		   + (61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D * D * D * D * D / 720);
	   //转换为度 DD
	   l = R2D(l);
	   b = R2D(b);
	   return  BLHCoordinate{ b, l, XYZ.z };
   }
};
#endif // !_COORDINATETRANSFORM_H_

