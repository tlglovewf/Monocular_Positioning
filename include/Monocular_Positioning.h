//
//  Monocular_Positioning.hpp
//  MonocularPositioning
//
//  Created by TuLigen on 2019/6/4.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef Monocular_Positioning_hpp
#define Monocular_Positioning_hpp
#include "Monocular_Utils.h"
#include "Monocular_Serialization.h"
#include "Monocular_Config.h"
class IDataLoader;

/*
 * 单目定位类
 */
class  Monocular_Positioning
{
public:
    Monocular_Positioning(const string &outputdir);
    ~Monocular_Positioning();
    

	/*
	 * 设置配置对象
	 */
	bool SetConfig(const std::shared_ptr<IConfig> &config);
    
    /*
     * 设置相机参数
     * @param cam 相机
     */
    void SetCamera(const Camera &cam)
    {
        mCamera = std::move(cam);
    }
    
    /*
     * 加载imu数据
     * @param path  数据路径
     */
    void LoadImuData(const string &path);
    
    /*
     * 根据图片名称获取imu数据
     * @param  picname 图片名称
     * @return         姿态数据
     */
    PoseData GetItemData(const std::string &picname);
    
    /*
     * 从R 和 t 中恢复 极线
     * @param R       帧间旋转矩阵
     * @param t       帧间平移矩阵
     * @param pixel   像素值
     * @param a,b,c   极线 ax + by + c = 0;
     */
    void  GetEpilineFromPose(const Mat &R, const Mat &t,
                             const Point2f &pixel,
                             double &a, double &b, double &c);
    
    /*
     * 极线匹配(基于目标匹配)
     * @param a,b,c     极线
     * @param targets   当前帧目标列表
     * @param target    待匹配目标
     */
    Target  EpilineMatch(double a, double b, double c,
                             const TargetVector &targets,
                             const Target &item);
    
    /*
     * 极线匹配(基于块匹配)
     * @param a,b,c     极线
     * @param preframe  前帧
     * @param curframe  后帧
     * @param pt        需要匹配的点像素
     * @param img       输出图像
     */
    Point2f EpilineMatch(double a, double b, double c,
                         const FrameData &preframe,
                         const FrameData &curframe,
                         const Point2f   &pt,
                         Mat &img);
    
    /*
     * 绘制极线
     * @param img   图片
     * @param pt    图像点
     * @param a,b,c 极线
     *
     */
    void OutputEpiline(Mat &img,
                       const Target &pre,const Target &cur,
                     double a, double b, double c);
    
    void OutputEpiline(Mat &img,
                       const Point2f &prept,const Point2f &curpt,
                       double a, double b, double c);
    /*
     * 定位
     * @param prept     前帧目标坐标
     * @param curpt     后帧目标坐标
     * @param predata   前帧imu数据
     * @param curdata   当前帧imu数据
     * @param R         旋转矩阵
     * @param t         平移矩阵
     * @return          经纬度
     */
    BLHCoordinate Position(const Point2f  &prept,
                           const Point2f  &curpt,
                           const PoseData &predata,
                           const PoseData &curdata,
                           Mat &R, Mat &t);
    
    /*
     * 定位
     * @param preframe  前帧
     * @param curframe  后帧
     * @param results   结果
     */
    bool Position(FrameData    &preframe,
                  FrameData    &curframe,
                  ResultVector &results);
    
    
    /*
     * 反投
     */
    Point2f BackProject(const FrameData &frame,const BLHCoordinate &blh, Mat &outimg);

	/*
	* 测试
	*/
	void Test();

	/*
	 * 存储图片
	 * @param path  存储路径
	 * @param img   需要存储的图片
	 * @param promt 文字
	 */
	void SaveImage(const std::string &path, Mat &img, const std::string &promt = "");
protected:
    Camera                          mCamera;
    std::unique_ptr<IDataLoader>    mpDataLoader;
    std::unique_ptr<ISerialization> mpSer;
    string                          mOutPutDir;
};

#endif /* Monocular_Positioning_hpp */
