//
//  Serialization.hpp
//  MonocularPositioning
//
//  Created by TuLigen on 2019/6/13.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef Serialization_hpp
#define Serialization_hpp
#include "Monocular_Types.h"
#include <iomanip>
#include <fstream>
/*
 *  序列化
 */
class ISerialization
{
public:
    /*
     * 格式化写经纬度
     * @param prompt 标签
     * @param geops  经纬度
     */
    virtual void writeFormat(const std::string &prompt,const BLHCoordinate &geopos) = 0;
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param value  float值
     */
    virtual void writeFormat(const std::string &prompt,float value) = 0;
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param mat    矩阵
     */
    virtual void writeFormat(const std::string &prompt,const Mat &mat) = 0;
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param rect   边框
     */
    virtual void writeFormat(const std::string &prompt,const Rect2f &rect) = 0;
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param point  点
     */
    virtual void writeFormat(const std::string &prompt,const Point2f &pt) = 0;
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param point  点
     */
    virtual void writeFormat(const std::string &prompt,const Point3d &pt) = 0;
    
    /*
     * 打印字符
     */
    virtual void prompt(const std::string &prompt) = 0;
    
	/*
	 * 空行
	 */
	virtual void space() = 0;

    /*
     * 写入
     */
    virtual void writeFlush() = 0;
};

/*
 * 帧文件序列化对象
 */
class FileSerialization : public ISerialization,public std::ofstream
{
public:
    FileSerialization(const std::string &path);
    ~FileSerialization()
    {
        close();
    }
    /*
     * 序列化
     */
    virtual void serialize(const FrameData &f);
    
    
    /*
     * 格式化写经纬度
     * @param prompt 标签
     * @param geops  经纬度
     */
    virtual void writeFormat(const std::string &prompt,const BLHCoordinate &geopos);
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param value  float值
     */
    virtual void writeFormat(const std::string &prompt,float value);
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param mat    矩阵
     */
    virtual void writeFormat(const std::string &promt,const Mat &mat);
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param rect   边框
     */
    virtual void writeFormat(const std::string &prompt,const Rect2f &rect);
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param point  点
     */
    virtual void writeFormat(const std::string &prompt,const Point2f &pt);
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param point  点
     */
    virtual void writeFormat(const std::string &prompt,const Point3d &pt) ;
    
    /*
     * 打印字符
     */
    virtual void prompt(const std::string &prompt);
    
	/*
	* 空行
	*/
	virtual void space();

    /*
     * 写入
     */
    virtual void writeFlush()
    {
        this->flush();
    }
    
protected:
    /*
     * 路径
     */
    std::string mPath;
};


#endif /* Serialization_hpp */
