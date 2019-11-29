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

#if 0
const string s_imgRoot = "/media/navinfo/Bak/Datas/wydatas/";

const string s_imgPath = s_imgRoot + "left/";
const string s_imgGray = s_imgRoot + "gray/";
const string s_excfg   = s_imgRoot + "/config/EX_0003_20190614.xml";
const string s_bscfg   = s_imgRoot + "/config/BS_0003_20190614.bs"; 
const string s_pstpath = s_imgRoot + "/20190615.PosT";

const float s_ImgDis = 4.0;
const int   s_stNo = 0;
#else

// const string s_imgRoot = "/media/navinfo/Bak/Datas/@@1002-0001-190828-00/Output/";

// const string s_imgPath = s_imgRoot + "LeftCamera/";
// const string s_imgGray = s_imgRoot + "gray/";
// const string s_excfg = s_imgRoot + "/extrinsics.xml";
// const string s_bscfg = s_imgRoot + "0001-1220.bs";
// const string s_pstpath = s_imgRoot + "/1002-0001-190828-00.PosT";
// const string s_imupath = s_imgRoot + "/imu190828_071742.txt";
// const float s_ImgDis = 1.0;
// const int s_stNo = ;0 //2140;


const string s_imgRoot = "/media/navinfo/Bak/Datas/@@1002-0001-191122-03/";

const string s_imgPath = s_imgRoot + "/";
const string s_imgGray = s_imgRoot + "gray/";
const string s_excfg = s_imgRoot + "/extrinsics.xml";
const string s_bscfg = s_imgRoot + "BS_0001_20191105.bs";
const string s_pstpath = s_imgRoot + "/1002-0001-191122-03-base.PosT";
const string s_imupath = s_imgRoot + "/imr.txt";
const float s_ImgDis = 1.0;
const int s_stNo = 1600;//1600; //2140;

#endif

const string s_o_rel = "/media/navinfo/Bak/CPP/QT/real.txt";
const string s_o_est = "/media/navinfo/Bak/CPP/QT/est.txt";
const string s_o_img = "/media/navinfo/Bak/CPP/QT/OutImg/";
void inline SwapFrame(Frame *&preframe, Frame *&curframe)
{
    assert(NULL != preframe);
    assert(NULL != curframe);

    delete preframe;
    preframe = curframe;
    curframe = NULL;
}

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
    // if(argc < 2)
    // {
    //     cout << "please input config file path ." << endl;
    //     return -1;
    // }
    // HandleOriginData(argv[1]);
    // return 0;
    cout.precision(20);
 
    ORB_SLAM2::IMUPreintegrator imupre;

    CDataManager::getSingleton()->LoadData(s_imgGray + "/pst.txt",s_imgGray + "/imu.txt");

    FrameDrawer *pFdrawer = NULL;
    Map *pMap = new Map();
    MapDrawer *pMpDrawer = new MapDrawer(pMap, "config");
    Viewer viewer(pFdrawer, pMpDrawer);

    thread thd(&Viewer::Run,&viewer);
    
    std::shared_ptr<IConfig> pConfig =
        std::make_shared<WeiYaConfig>(s_excfg, s_bscfg);

    Camera cam;
    pConfig->ReadConfig(cam);

    Ptr<IFeatureTrack> track = new ORBFeatureTrack();

    Frame *preFrame = new Frame;
    Frame *curFrame = NULL;
    CMap map;

    const int bg_no = s_stNo;

    ImgInfoVIter bgIter = CDataManager::getSingleton()->begin() + bg_no;
    ImgInfoVIter edIter = CDataManager::getSingleton()->end();

    Mat preimg = imread(s_imgGray + bgIter->first , CV_LOAD_IMAGE_UNCHANGED);

    preFrame->setImage(preimg);

    track->calcFeatures(preimg, preFrame);

    preFrame->initStaticParams();

    double pretime = M_Untils::GetDayTimeFromPicName(bgIter->first);

    cout.precision(15);

    ofstream realfile;
    ofstream estfile;

    realfile.open(s_o_rel);
    estfile.open(s_o_est);
    realfile.precision(15);
    estfile.precision(15);

    CDataManager::getSingleton()->setIndicator(bg_no);

#if 0
    for(; bgIter != edIter; ++bgIter)
    {
        writeRealTrace(realfile,bgIter->second.pos,bgIter->first.substr(28,4));
        // if(bgIter->second.pos.longitude > 114.44)
        {
            cout << "------------------" << bgIter->second.pos.longitude << " " << bgIter->second.pos.latitude << endl;
        }
    }
#endif

    BLHCoordinate espos;

    PoseData orignPose;

    Mat t_R;
    Mat t_T;

    Mat r_R;
    Mat r_T;

#define WIN_SIZE 1000
    Mat traj = Mat::zeros(WIN_SIZE, WIN_SIZE, CV_8UC3);

    CDataManager::getSingleton()->setIndicator(bg_no);
    Eigen::Vector3d total(0,0,0);
    int index = 0;
    Mat result_t;
    for ( ++bgIter; bgIter != edIter; ++bgIter)
    {
        ++index;
        IMURawVector imudatas = CDataManager::getSingleton()->getIMUDataFromLastTime(bgIter->second._t);
        // cout << "imu data size : " << imudatas.size() << endl;
        double last_t ;//= (bgIter - 1)->second._t;
        cout << "img " << bgIter->first.c_str() << endl;
        // for(size_t i = 0; i < imudatas.size(); ++i)
        // {
        //     if( i == imudatas.size() - 1)
        //     {
        //         last_t = bgIter->second._t;
        //     }
        //     else
        //     {
        //         last_t = imudatas[i + 1]._t;
        //     }

        //     Eigen::Vector3d gyro(imudatas[i]._gyro_x,imudatas[i]._gyro_y,imudatas[i]._gyro_z);
        //     Eigen::Vector3d acc(imudatas[i]._acc_x,imudatas[i]._acc_y,imudatas[i]._acc_z);
        //     total += acc - Eigen::Vector3d(0,1.0,9.8);
        //     // if(acc.y() < 0)
        //     // {
        //     //     cout << "total " << total << endl;
        //     // }
        
        //     double dt = last_t - imudatas[i]._t;
        //     // imupre.update(gyro, acc, dt);
        //     imupre.predict(acc,gyro,dt);
        // }
        // cout <<  "Velocity : " << (total) / index << endl;
        Mat curimg = imread(s_imgGray + bgIter->first, CV_LOAD_IMAGE_UNCHANGED);
        curFrame = new Frame;
        curFrame->setImage(curimg);

        track->calcFeatures(curimg, curFrame);

        Mat matchimg;
        MatchVector mt;

        track->match(preFrame, curFrame, mt);

        if (curFrame->getCurMatchPts().size() < 10)
        {
            printf("there has no rigt data for next step...  %s \n", bgIter->first.c_str());
            preFrame = curFrame;
            curFrame = NULL;
            continue;
        }

        Mat E, R, t, mask;

        E = findEssentialMat(preFrame->getPreMatchPts(), curFrame->getCurMatchPts(), cam.K, RANSAC,
                             0.999, 1.0, mask);
        recoverPose(E, preFrame->getPreMatchPts(), curFrame->getCurMatchPts(), cam.K, R, t, mask);

        double ln = t.at<double>(0, 0) * t.at<double>(0, 0) +
                    t.at<double>(1, 0) * t.at<double>(1, 0) +
                    t.at<double>(2, 0) * t.at<double>(2, 0);
        ln = s_ImgDis / sqrt(ln);

        t = t * ln;

        PoseData prepose = (bgIter - 1)->second;
        double curtime = M_Untils::GetDayTimeFromPicName(s_imgGray + bgIter->first);
        PoseData curpose = bgIter->second;
        // cout << "pose " << curpose._roll << " " << curpose._pitch << " " << curpose._yaw << endl;
        cout << "pose " << curpose.pos.longitude << " " << curpose.pos.latitude << endl;
        static bool bol = false;
        cv::Mat preMat = M_CoorTrans::IMU_to_ENU(-prepose._yaw,prepose._pitch,prepose._roll);
        cv::Mat curMat = M_CoorTrans::IMU_to_ENU(-curpose._yaw,curpose._pitch,curpose._yaw);

        cv::Mat dR;
        cv::Mat dT;

        M_Untils::GetRtFromPose(prepose, curpose, cam.RCam2Imu, cam.TCam2Imu,dR, dT);

        if (!bol)
        {
            orignPose = prepose;
            t_R = R.clone();
            t_T = t.clone();
            r_R = dR.clone();
            r_T = dT.clone();
            result_t = -R.inv() * t_T;
        }
        else
        {
   
            t_R = R * t_R;
            t_T = R * t_T + t;

            r_R = dR * r_R;
            r_T = dR * r_T + dT;

            result_t = -t_R.inv() * t_T;
        }

        // writeRealTrace(realfile, prepose.pos, bgIter->first);
        const PtVector &curpts = curFrame->getCurMatchPts();

        char ottxt[255] = {0};
        
        preFrame = curFrame;
        map.push(curFrame);

        curFrame = NULL;

        BLHCoordinate _blh;

        M_Untils::CalcPoseFromRT(orignPose, Mat::eye(3, 3, CV_64F), result_t, cam.RCam2Imu, cam.TCam2Imu, _blh);
        //计算高斯投影差值
        Point3d realgauss = M_CoorTrans::BLH_to_GaussPrj(curpose.pos);
        Point3d estgauss = M_CoorTrans::BLH_to_GaussPrj(_blh);

        Point3d residual = estgauss - realgauss;

        // writeEstTrace(estfile, _blh, residual, bgIter->first);

        //system("free -h");
        Mat t_t = Mat::eye(4, 4, CV_64F);
        t_R.copyTo(t_t.rowRange(0, 3).colRange(0, 3)); //

        t_T.copyTo(t_t.rowRange(0, 3).col(3));

        pMpDrawer->SetCurrentCameraPose(t_t);

        KeyFrame *kframe = new KeyFrame();
        kframe->SetWorldPos(t_t);
        pMap->push(kframe);


        r_R.copyTo(t_t.rowRange(0, 3).colRange(0, 3)); //
        r_T.copyTo(t_t.rowRange(0, 3).col(3));
        kframe = new KeyFrame();
        kframe->SetWorldPos(t_t);
        pMap->pushReal(kframe);
        // cout << "imu pre pose : " << imupre.getDeltaP() << endl;
        // cout << "v   get pose : " << result_t << endl;
        // pMap->insertIMUPose(imupre.getDeltaP());
       
       cv::Mat mat;
        resize(curimg,mat,cv::Size(curimg.cols >> 2, curimg.rows >> 2));


        cv::imshow("test",mat);
        cv::waitKey(1);   

        pretime = curtime;
        if(!bol)
        {
            thd.detach();
            bol = true;
        }
    }

//     getchar();
    realfile.close();
    estfile.close();

    return 0;
}
