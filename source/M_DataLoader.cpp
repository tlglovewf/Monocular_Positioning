//
//  DataLoader.cpp
//  MonocularPositioning
//
//  Created by TuLigen on 2019/6/10.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#include "M_DataLoader.h"
#include <fstream>
#include <algorithm>
#include <ctime>
#include "M_Utils.h"
using namespace std;

#define ISREAL 1          // 是否取真值(post文件)
#define POSTHEADLINENO 50 //post头文件指定长度

#define POSTMAXREADLENGTH 1e5
#define IMURAWMAXREADLENGTH 3e5

/*
 *  加载数据
 *  @param  path 路径
 *  @return      姿态集合
 */
PoseVector &Stim300PostTDataLoader::loadData(const std::string &path)
{
    ifstream file;
    try
    {
        file.open(path);

        Time_Interval interval;
        interval.start();

        cout << "stim300 pst datas begin loading !!!" << endl;

        if (file.is_open())
        {
            string str;

#if ISREAL
            int i = 0;
            while (i++ < POSTHEADLINENO)
            {
                getline(file, str); // read header
            }
#endif
            int nidx;
            double psttime;
            double north;
            double east;
            float hell;
            double lat;
            double lon;
            float hzspeed;
            double pitch;
            double roll;
            double yaw;
            char other[255] = {0};

            int index = 0;

            while (!file.eof()) // && ++index < POSTMAXREADLENGTH)
            {
                PoseData item;
                getline(file, str);
#if ISREAL
                sscanf(str.c_str(), "%d %lf %lf %lf %f %lf %lf %f %lf %lf %lf %s",
                       &nidx, &psttime, &north, &east, &hell, &lat, &lon, &hzspeed, &roll, &pitch, &yaw, other);
#else
                sscanf(str.c_str(), "%lf %lf %lf %f %lf %lf %f %lf %lf %lf",
                       &psttime, &north, &east, &hell, &lat, &lon, &hzspeed, &roll, &pitch, &yaw);
#endif
                item._t = psttime;
                item.pos.longitude = lon;
                item.pos.latitude = lat;
                item.pos.altitude = hell;

#define POSITIVENUM(x) (x) //  (x < 0) ? (360 + x) : x

                item._pitch = POSITIVENUM(pitch);
                item._roll = POSITIVENUM(roll);
                item._yaw = POSITIVENUM(yaw);

                mPoseDatas.emplace_back(item);
            }
            std::cout << mPoseDatas.size() << "pst datas load successfully!!!" << std::endl;
            interval.prompt("It takes ");
        }
        resetIndicator();
    }
    catch (...)
    {
        file.close();
    }

    return mPoseDatas;
}
template <typename T>
class Cmp
{
public:
    Cmp(double t) : _t(t) {}

    bool operator()(const T &data)
    {
        return data._t > _t;
    }

protected:
    double _t;
};

PoseData Stim300PostTDataLoader::getData(double t)
{
    Time_Interval interval;
    interval.start();

    //从指针处开始搜索
    PoseVIter it = find_if(mIndicator, mPoseDatas.end(), Cmp<PoseData>(t));

    if (it != mPoseDatas.end())
    {
        int index = static_cast<int>(it - mPoseDatas.begin());
        //        cout << "Get pose index : " << index  << " time : " << mPoseDatas[index]._t <<  endl;
        assert(index > 0);

        const PoseData &curdata = mPoseDatas[index];
        const PoseData &predata = mPoseDatas[index - 1];

        double lon =
            M_Untils::GetLxValue(t, predata._t, curdata._t, predata.pos.longitude, curdata.pos.longitude);

        double lat =
            M_Untils::GetLxValue(t, predata._t, curdata._t, predata.pos.latitude, curdata.pos.latitude);

        double h =
            M_Untils::GetLxValue(t, predata._t, curdata._t, predata.pos.altitude, curdata.pos.altitude);

        double pitch =
            M_Untils::GetLxValue(t, predata._t, curdata._t, predata._pitch, curdata._pitch);

        double roll =
            M_Untils::GetLxValue(t, predata._t, curdata._t, predata._roll, curdata._roll);

        double yaw =
            M_Untils::GetLxValue(t, predata._t, curdata._t, predata._yaw, curdata._yaw);

        //        interval.prompt("Getting posedata costs ");

        mIndicator = it; //设置指针

        return PoseData{t, {lat, lon, h}, pitch, roll, yaw};
    }
    else
    {
        //数据中查询不到,返回一个错误值
        return PoseData{WRONGDATA};
    }
}

PoseVector &Imu5651DataLoader::loadData(const std::string &path)
{
    return mPoseDatas;
}
/* 加载数据
     * @param path 数据路径
    */
bool STIM300IMURawDataLoader::loadData(const std::string &path)
{
    if (!path.empty())
    {
        ifstream fs;
        try
        {
            fs.open(path);
            cout << "begin loading imu raw data." << endl;
            std::string txtline;

            //get headline
            getline(fs, txtline);
            int index = 0;
            while (!fs.eof()) // && (index++ < IMURAWMAXREADLENGTH))
            {
                getline(fs, txtline);
                double t;
                double gyro_x;
                double gyro_y;
                double gyro_z;
                double acc_x;
                double acc_y;
                double acc_z;
                sscanf(txtline.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &t, &gyro_x, &gyro_y, &gyro_z,
                       &acc_x, &acc_y, &acc_z);
                ImuRawData data = {M_Untils::Wktime2Daytime(t), gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z};
                mDatas.emplace_back(data);
            }
            mIndicator = mDatas.begin();
            fs.close();
            cout << "load imu raw data successfully!!!" << endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    return false;
}

/* 获取数据
     * @param second 时间
     * @return imu原始数据
    */
ImuRawData STIM300IMURawDataLoader::getData(double second)
{
    const IMURawVIter iter = std::find_if(mIndicator, mDatas.end(), Cmp<ImuRawData>(second));
    
    return getRawData(second, iter);
}

/* 获取数据
     * @param bgsec 开始时间 
     * @param edsec 结束时间
     * @return      时间间隔数据集
    */
IMURawVector STIM300IMURawDataLoader::getDatas(double bgsec, double edsec)
{
    IMURawVector tmp;

    const IMURawVIter bgiter = std::find_if(mIndicator, mDatas.end(), Cmp<ImuRawData>(bgsec));
    const IMURawVIter editer = std::find_if(mIndicator, mDatas.end(), Cmp<ImuRawData>(edsec));

    //first data
    tmp.emplace_back(getRawData(bgsec, bgiter));
    for (IMURawVIter it = bgiter; it < editer; ++it)
    {
        tmp.emplace_back(*it);
    }
    //last data
    tmp.emplace_back(getRawData(edsec,editer));

    return tmp;
}

/*  获取数据
     */
ImuRawData STIM300IMURawDataLoader::getRawData(double sec, const IMURawVIter &it)
{
    int index = static_cast<int>(it - mDatas.begin());

    assert(index < mDatas.size() && index > 0);
    mIndicator = it;
    const ImuRawData &predata = mDatas[index - 1];
    const ImuRawData &nxtdata = mDatas[index];

    cout.precision(20);

    cout << predata._t << endl;
    cout << nxtdata._t << endl;

    //拉格朗日差值
    double gx = M_Untils::GetLxValue(sec, predata._t, nxtdata._t, predata._gyro_x, nxtdata._gyro_x);
    double gy = M_Untils::GetLxValue(sec, predata._t, nxtdata._t, predata._gyro_y, nxtdata._gyro_y);
    double gz = M_Untils::GetLxValue(sec, predata._t, nxtdata._t, predata._gyro_z, nxtdata._gyro_z);

    double ax = M_Untils::GetLxValue(sec, predata._t, nxtdata._t, predata._acc_x, nxtdata._acc_x);
    double ay = M_Untils::GetLxValue(sec, predata._t, nxtdata._t, predata._acc_y, nxtdata._acc_y);
    double az = M_Untils::GetLxValue(sec, predata._t, nxtdata._t, predata._acc_z, nxtdata._acc_x);

    //cout << "get data : " << sec << " " << gx << " " << gy << " " << gz << " " << ax << " " << ay << " " << az << endl;
    return ImuRawData(sec, gx, gy, gz, ax, ay, az);
}