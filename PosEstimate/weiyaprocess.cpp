//
//  main.cpp
//  PosEstimate
//
//  Created by TuLigen on 2019/8/9.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#include <fstream>
#include <algorithm>
#include "M_Utils.h"
#include "CFuncHelper.h"
#include "M_CoorTransform.h"
#include "M_Config.h"
#include "M_DataLoader.h"
#include "CFeatureTrack.h"
#include "COptimizer.h"
#include "CFrame.h"
#include "CMap.h"
#include "CViewer.h"
#include <thread>
#include "algorithm"

#include "CDataManager.h"

#include "IMU/IMUPreintegrator.h"

using namespace cv;
using namespace std;

//convert rgb img to  gray img
void conver2Gray(const std::string &inputpath, const std::string &outpath)
{
    if (inputpath.empty() ||
        outpath.empty())
    {
        return;
    }
    else
    {
        FileNameVec filenms;
        loadFiles(inputpath, filenms);

        for (size_t i = 0; i < filenms.size(); ++i)
        {
            Mat img = imread(filenms[i], CV_LOAD_IMAGE_UNCHANGED);
            int n = filenms[i].find_last_of('/');
            std::string outfilename = outpath + filenms[i].substr(++n);

            Mat grayimg;
            cvtColor(img, grayimg, CV_RGB2GRAY);
            printf("%s %d - %d \n", outfilename.c_str(), grayimg.channels(), grayimg.type());
            imwrite(outfilename, grayimg);
        }
    }
}

#define FTLEN 25
#define WRITESPACE(FILE) FILE.width(FTLEN);
#define WRITEPOSEDATA(FILE, NAME, TIME, LON, LAT, ALT, PITCH, YAW, ROLL, V) \
    FILE.width(FTLEN * 2);                                               \
    FILE << NAME;                                                        \
    WRITESPACE(FILE)                                                     \
    FILE << TIME;                                                        \
    WRITESPACE(FILE)                                                     \
    FILE << LON;                                                         \
    WRITESPACE(FILE)                                                     \
    FILE << LAT;                                                         \
    WRITESPACE(FILE)                                                     \
    FILE << ALT;                                                         \
    WRITESPACE(FILE)                                                     \
    FILE << PITCH;                                                       \
    WRITESPACE(FILE)                                                     \
    FILE << YAW;                                                         \
    WRITESPACE(FILE)                                                     \
    FILE << ROLL;                                                        \
    WRITESPACE(FILE)                                                     \
    FILE << V;                                                           \
    FILE << endl;

#define WRITEIMURAWDATA(FILE, TIME, GX, GY, GZ, AX, AY, AZ) \
    WRITESPACE(FILE);                                       \
    FILE << TIME;                                           \
    WRITESPACE(FILE);                                       \
    FILE << GX;                                             \
    WRITESPACE(FILE);                                       \
    FILE << GY;                                             \
    WRITESPACE(FILE);                                       \
    FILE << GZ;                                             \
    WRITESPACE(FILE);                                       \
    FILE << AX;                                             \
    WRITESPACE(FILE);                                       \
    FILE << AY;                                             \
    WRITESPACE(FILE);                                       \
    FILE << AZ;                                             \
    FILE << endl;

/* 预处理数据
 * @param imgpath  图片路径
 * @param pstpath  pst路径
 * @param imupath  imu路径
 * @param outpath  输出路径
 * @param oimpath  imu输出路径
 */
void PreprocessData(const std::string &imgpath,
                    const std::string &oimgpath,
                    const std::string &pstpath,
                    const std::string &imupath,
                    const std::string &outpath,
                    const std::string &oimpath)
{
    // assert(!imgpath.empty() && pstpath.empty() && !imupath.empty() && !outpath.empty() && !oimpath.empty());

    ofstream pFile, pImu;
    pFile.precision(15);
    pFile.flags(ios::left | ios::fixed);

    pImu.precision(15);
    pImu.flags(ios::left | ios::fixed);

    try
    {
        pFile.open(outpath);
        pImu.open(oimpath);
        FileNameVec fms;
        int sz = loadFiles(imgpath, fms);

        Ptr<IDataLoader> pPstDataLoader = new Stim300PostTDataLoader();
        pPstDataLoader->loadData(pstpath);

        Ptr<IMURawDataLoader> pIMUDataloader = new STIM300IMURawDataLoader();
        pIMUDataloader->loadData(imupath);

        cout << "begin writing datas." << endl;
        WRITEPOSEDATA(pFile, "Name", "Time", "Lon", "Lat", "Alt", "Pitch", "Yaw", "Roll","Vel");
        WRITEIMURAWDATA(pImu, "Time", "Gyro_x", "Gyro_y", "Gyro_z", "Acc_x", "Acc_y", "Acc_z");

        double bgtime = 0.0;
        double edtime = 0.0;

     
        for (int i = 0; i < sz; ++i)
        {
            const std::string &imgname = fms[i].substr(fms[i].length() - 38);
            const double imgtime = M_Untils::GetDayTimeFromPicName(imgname);
            Mat img = imread(fms[i], CV_LOAD_IMAGE_UNCHANGED);
            std::string outfilename = oimgpath + imgname;

            Mat grayimg;
            cvtColor(img, grayimg, CV_RGB2GRAY);
            printf("%s %d\n", outfilename.c_str(), grayimg.channels());
            imwrite(outfilename, grayimg);


            const PoseData posedata = pPstDataLoader->getData(imgtime);

            WRITEPOSEDATA(pFile, imgname, posedata._t,
                          posedata.pos.longitude,
                          posedata.pos.latitude,
                          posedata.pos.altitude,
                          posedata._pitch,
                          posedata._yaw,
                          posedata._roll,
                          posedata._v);

            if (i > 0)
            {//from second frame
                edtime = imgtime;
                IMURawVector rawdatas(pIMUDataloader->getDatas(bgtime, edtime));
                const int sti = (i == 1) ? 0 : 1;
                for (size_t i = sti; i < rawdatas.size(); ++i)
                {
                    WRITEIMURAWDATA(pImu, rawdatas[i]._t,
                                    rawdatas[i]._gyro_x,
                                    rawdatas[i]._gyro_y,
                                    rawdatas[i]._gyro_z,
                                    rawdatas[i]._acc_x,
                                    rawdatas[i]._acc_y,
                                    rawdatas[i]._acc_z);
                }
                bgtime = edtime;
            }
            else
            {
                bgtime = imgtime;
            }
        }
        pFile.close();
        pImu.close();
        cout << "write successfully!!!" << endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

/* 处理原始数据
*/
void HandleOriginData(const string &path)
{

    cv::FileStorage fSetting(path,cv::FileStorage::READ);

    std::string rootpath;
    std::string imagepath;
    std::string outputpath;
    std::string postfilepath;
    std::string imurawpath;
    std::string pstpath;
    std::string imupath;
    
    fSetting["RootPath"]     >> rootpath;
    fSetting["ImagePath"]    >> imagepath;
    fSetting["OutPutPath"]   >> outputpath;
    fSetting["PosTFilePath"] >> postfilepath;
    fSetting["ImuRawPath"]   >> imurawpath;
    fSetting["PstPath"]      >> pstpath;
    fSetting["ImuPath"]      >> imupath;
    cout << "RootPath " << rootpath.c_str() << endl;
    cout << "ImagePath " << rootpath.c_str() << endl;
    cout << "OutPutPath " << rootpath.c_str() << endl;
    cout << "RootPath " << rootpath.c_str() << endl;

    PreprocessData( rootpath + imagepath,
                    rootpath + outputpath,
                    rootpath + postfilepath,
                    rootpath + imurawpath,
                    rootpath + pstpath,
                    rootpath + imupath  );

}


// main func
int main(int argc, const char *argv[])
{
    if(argc < 2)
    {
        cout << "please input config file path ." << endl;
        return -1;
    }
    HandleOriginData(argv[1]);
    return 0;
}
