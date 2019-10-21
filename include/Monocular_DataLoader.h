//
//  DataLoader.hpp
//  MonocularPositioning
//
//  Created by TuLigen on 2019/6/10.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#ifndef DataLoader_hpp
#define DataLoader_hpp

#include "Monocular_Types.h"

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
    virtual bool loadDatas(const std::string &path) = 0;

    /* 获取数据
     * @param second 时间
     * @return imu原始数据
    */
    virtual ImuRawData getDatas(double second) = 0;
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
    virtual bool loadDatas(const std::string &path) ;

    /* 获取数据
     * @param second 时间
     * @return imu原始数据
    */
    virtual  ImuRawData getDatas(double second);

protected:
    IMURawVector mDatas;
    IMURawVIter  mIndicator;
};








#endif /* DataLoader_hpp */
