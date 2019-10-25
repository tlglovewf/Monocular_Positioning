//
//  DataLoader.hpp
//  
//
//  Created by TuLigen on 2019/6/10.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef _M_DATALOADER_H_H_
#define _M_DATALOADER_H_H_

#include "M_Types.h"

/*
 *  数据加载类
 */
class  IDataLoader
{
public:
    IDataLoader(){}
    virtual ~IDataLoader(){}
    
    /*
     *  加载数据
     *  @param  path 路径
     *  @return      姿态集合
     */
    virtual PoseVector& loadData(const std::string &path) = 0;
    
    /*
     *  获取数据
     *  @param  t    时间
     *  @return      姿态数据
     */
    virtual PoseData getData(double t) = 0;
    
    /*
     * 重置指针
     */
    virtual void resetIndicator()
    {
        mIndicator = mPoseDatas.begin();
    }
protected:
    PoseVector mPoseDatas;
    PoseVIter  mIndicator;
};

/*
 *   stim300 数据加载类
 */
class Stim300PostTDataLoader : public IDataLoader
{
public:
    /*
     *  加载数据
     *  @param  path 路径
     *  @return      姿态集合
     */
    virtual PoseVector& loadData(const std::string &path);
    
    /*
     *  获取数据
     *  @param  t    时间
     *  @return      姿态数据
     */
    virtual PoseData   getData(double t);
};

/*
 *    5651 数据加载类
 */
class Imu5651DataLoader : public IDataLoader
{
public:
    /*
     *  加载数据
     *  @param  path 路径
     *  @return      姿态集合
     */
    virtual PoseVector& loadData(const std::string &path);
};


/*
    Imu 数据读取
*/
class IMURawDataLoader
{
public:
    IMURawDataLoader(){}
    virtual ~IMURawDataLoader(){}

    /* 加载数据
     * @param path 数据路径
    */
    virtual bool loadData(const std::string &path) = 0;

    /* 获取数据
     * @param second 时间
     * @return imu原始数据
    */
    virtual ImuRawData getData(double second) = 0;

    /* 获取数据
     * @param bgsec 开始时间 
     * @param edsec 结束时间
     * @return      时间间隔数据集
    */
    virtual IMURawVector getDatas(double bgsec, double edsec) = 0;
protected:
    IMURawVector mRawDatas;
};

/* stim300 数据读取
*/
class STIM300IMURawDataLoader : public IMURawDataLoader
{
public:
    /* 加载数据
     * @param path 数据路径
    */
    virtual bool loadData(const std::string &path) ;

    /* 获取数据
     * @param second 时间
     * @return imu原始数据
    */
    virtual  ImuRawData getData(double second);

    /* 获取数据
     * @param bgsec 开始时间 
     * @param edsec 结束时间
     * @return      时间间隔数据集
    */
    virtual IMURawVector getDatas(double bgsec, double edsec);

protected:
    /*  获取数据
     */
    ImuRawData getRawData(double sec, const IMURawVIter &it); 

protected:
    IMURawVector mDatas;
    IMURawVIter  mIndicator;
};








#endif /* DataLoader_hpp */
