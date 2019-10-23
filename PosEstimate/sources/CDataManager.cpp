//
//  CDataManager.cpp
//
//  Created by TuLigen on 2019/10/22.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#include "CDataManager.h"

CDataManager *CDataManager::getSingleton()
{
    static CDataManager instance;
    return &instance;
}

/*
     * 加载数据
     * @param pstpath 预处理后的imgpst 文件
     * @param imupath 预处理后的imu 文件
     */
bool CDataManager::LoadData(const string &pstpath, const string &imupath)
{
    if (pstpath.empty() || imupath.empty())
    {
        return false;
    }
    else
    {
        ifstream pstfile, imufile;

        try
        {
            cout << "开始加载文件数据" << endl;
            pstfile.open(pstpath);
            imufile.open(imupath);

            std::string pststr;
            std::string imustr;

            //read headline
            getline(pstfile, pststr);
            getline(imufile, imustr);

            //load img pst file
            while (!pstfile.eof())
            {
                getline(pstfile, pststr);
                char filename[255] = {0};
                PoseData pose;
                sscanf(pststr.c_str(),"%s %lf %lf %lf %lf %lf %lf %lf", filename,
                                                                            &pose._t,
                                                                            &pose.pos.longitude,
                                                                            &pose.pos.latitude,
                                                                            &pose.pos.altitude,
                                                                            &pose._pitch,
                                                                            &pose._yaw,
                                                                            &pose._roll);

                mPoseDatas.emplace_back(std::make_pair(filename, pose));
            }

            //load imu file
            while (!imufile.eof())
            {
                getline(imufile, imustr);
                ImuRawData rawdata;
                sscanf(imustr.c_str(),"%lf %lf %lf %lf %lf %lf %lf", &rawdata._t,
                                                                     &rawdata._gyro_x,
                                                                     &rawdata._gyro_y,
                                                                     &rawdata._gyro_z,
                                                                     &rawdata._acc_x,
                                                                     &rawdata._acc_y,
                                                                     &rawdata._acc_z);
                mIMURawDatas.emplace_back(rawdata);
            }

            pstfile.close();
            imufile.close();

            cout << "文件加载成功" << endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        //数据加载成功后 设置imu文件游标
        mIMURawIndicator = mIMURawDatas.begin();

        return true;
    }

    return false;
}

//判断浮点型　相等
static inline bool isSame(double pre, double cur)
{
    return fabs(cur - pre) < 1e-6;
}

class Cmp
{
public:
    Cmp(double t):_t(t){}

    bool operator()(const ImuRawData &data)
    {
        return isSame(data._t,_t);
    }
private:
    double _t;
};

/* 当前时间到上个时间段内的imu数据集合
     * @param cursec(天秒)
     * @return 数据集
     */
IMURawVector CDataManager::getIMUDataFromLastTime(double cursec)
{
    //断言　imu数据集不能为空，且传入的时间不能与上次时间一致
    assert(!mIMURawDatas.empty() && !isSame(mIMURawIndicator->_t,cursec));

    IMURawVIter iter = std::find_if(mIMURawIndicator,mIMURawDatas.end(),Cmp(cursec));

    cout << iter->_t << endl;

    IMURawVector tmp;
    tmp.reserve(iter - mIMURawIndicator);

    tmp.assign(mIMURawIndicator,iter);

    return tmp;
}