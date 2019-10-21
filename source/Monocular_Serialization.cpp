//
//  Serialization.cpp
//  MonocularPositioning
//
//  Created by TuLigen on 2019/6/13.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#include "Monocular_Serialization.h"

//格式输出
#define WRITE_INFO(os,prompt,x)  os << prompt << ":" << (x) << std::endl;

FileSerialization::FileSerialization(const std::string &path)
{
    mPath = path + "/result.txt";
    assert(!path.empty());
    this->open(mPath);
    this->setf(std::ios::left);
    this->width(25);
    this->precision(25);
}

void FileSerialization::writeFormat(const std::string &prompt,const BLHCoordinate &geopos)
{
    assert(is_open());
    *this  << std::setprecision(20) << prompt.c_str()\
    << ":" << geopos.latitude  << ","
           << geopos.longitude << ","
           << geopos.altitude  << std::endl;
}

/*
 * 写经纬度
 * @param prompt 标签
 * @param value  float值
 */
void FileSerialization::writeFormat(const std::string &prompt,float value)
{
    assert(is_open());
    WRITE_INFO(*this,prompt.c_str(),value);
}

/*
 * 格式化写值
 * @param prompt 标签
 * @param mat    矩阵
 */
void FileSerialization::writeFormat(const std::string &prompt,const Mat &mat)
{
    assert(is_open());
    *this << prompt.c_str() << std::endl << mat << std::endl;
}

/*
 * 格式化写值
 * @param prompt 标签
 * @param rect   边框
 */
void FileSerialization::writeFormat(const std::string &prompt,const Rect2f &rect)
{
    assert(is_open());
    WRITE_INFO(*this,prompt.c_str(),rect);
}
/*
 * 打印字符
 */
void FileSerialization::prompt(const std::string &prompt)
{
    assert(is_open());
    *this << prompt.c_str() << ":" << std::endl;
}

/*
 * 格式化写值
 * @param prompt 标签
 * @param point  点
 */
void FileSerialization::writeFormat(const std::string &prompt,const Point2f &pt)
{
    assert(is_open());
    *this << prompt.c_str() << " : " << pt << std::endl;
}

/*
 * 格式化写值
 * @param prompt 标签
 * @param point  点
 */
void FileSerialization::writeFormat(const std::string &prompt,const Point3d &pt)
{
    assert(is_open());
    *this << prompt.c_str() << " : " << pt << std::endl;
}


/*
* 空行
*/
void FileSerialization::space()
{
	assert(is_open());
	*this << std::endl;
}

void FileSerialization::serialize(const FrameData &f)
{
    if(is_open())
    {
        //输出基本信息
        const BLHCoordinate &geo = f._pose.pos;
        writeFormat("pos", geo);//当前帧经纬度
        
        const TargetVector &items = f._targets;
        
        
        //输出目标信息
        for(const Target &item :items)
        {
            WRITE_INFO(*this,"type",item._type)
            WRITE_INFO(*this,"box",item._box);
            WRITE_INFO(*this,"center",item.center());
        }
        
    }
}
