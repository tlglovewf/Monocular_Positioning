//
//  main.cpp
//  PosEstimate
//
//  Created by TuLigen on 2019/8/9.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#include <fstream>
#include <algorithm>
#include "Monocular_Utils.h"
#include "FuncHelper.h"
#include "Monocular_CoorTransform.h"
#include "Monocular_Config.h"
#include "Monocular_DataLoader.h"
#include "FeatureTrack.h"
#include "COptimizer.h"
#include "CFrame.h"


using namespace cv;
using namespace std;

//const string s_imgPath = "/media/navinfo/Bak/Datas/wydatas/config/";


const string s_imgPath = "/media/navinfo/Bak/Datas/@@1002-0001-190828-00/Output/LeftCamera/";


const string s_excfg   = s_imgPath + "/EX_0003_20190614.xml";
const string s_bscfg   = s_imgPath + "/BS_0003_20190614.bs"; 
const string s_pstpath = s_imgPath + "/../20190615.PosT";

const string s_o_rel = "/media/navinfo/Bak/CPP/QT/real.txt";
const string s_o_est = "/media/navinfo/Bak/CPP/QT/est.txt";

int main(int argc, const char * argv[]) {
    // insert code here...
    std::shared_ptr<IConfig>  pConfig =
    std::make_shared<WeiYaConfig>(s_excfg,
                                  s_bscfg);
    
    Camera cam;
    pConfig->ReadConfig(cam);
    unique_ptr<IDataLoader> pData = std::make_unique<Stim300PostTDataLoader>();

    FileNameVec flnms;
    loadFiles(s_imgPath + "../leftd/", flnms);
    pData->loadData( s_pstpath );
    sort(flnms.begin(),flnms.end());
    FeatureTrack track;
  
    CFrame *preFrame = NULL;
    CFrame *curFrame = NULL;

    Mat preimg = imread(flnms[0]);
  
    KeyPtVector prekeys;
    Mat         predes;
    
    track.calcFeatures(preimg, prekeys, predes);
  
    double pretime = M_Untils::GetDayTimeFromPicName(flnms[0]);
    
    cout.precision(15);
    
    ofstream realfile;
    ofstream estfile;
    
    realfile.open(s_o_rel);
    estfile.open(s_o_est);
    realfile.precision(15);
    estfile.precision(15);
    BLHCoordinate espos;
    
    PoseData orignPose;
    
    Mat t_R;
    Mat t_T;
    
#define WIN_SIZE 1000
    Mat traj = Mat::zeros(WIN_SIZE, WIN_SIZE, CV_8UC3);
    
    for(size_t i = 1; i < flnms.size() ; ++i)
    {
        size_t n = flnms[i].find_last_of('-');
        const string fname = flnms[i].substr(++n);
        
        Mat curimg = imread(flnms[i]);
        
        KeyPtVector curkeys;
        Mat         curdes;
        
        track.calcFeatures(curimg, curkeys, curdes);
        
        PtVector prepts;
        PtVector curpts;
        
        track.knn_match(prekeys, curkeys, predes, curdes, prepts, curpts);
        
        prekeys.swap(curkeys);
        predes = curdes;
        
        Mat E, R, t, mask;
        E = findEssentialMat( curpts,prepts, cam.K,RANSAC,
                             0.999, 1.0, mask);
        recoverPose(E,  curpts, prepts,cam.K,R, t, mask);
        
        double ln = t.at<double>(0,0) * t.at<double>(0,0) +
        t.at<double>(1,0) * t.at<double>(1,0) +
        t.at<double>(2,0) * t.at<double>(2,0);
        ln = 4 / sqrt(ln);

        t = t * ln;
        
        PoseData prepose = pData->getData(pretime);
        double curtime =  M_Untils::GetDayTimeFromPicName(flnms[i]);
        PoseData curpose = pData->getData(curtime);
        if( 1 == i)
        {
            realfile << prepose.pos.latitude << "," << prepose.pos.longitude << "," << fname.c_str() << endl;
            estfile << prepose.pos.latitude << "," << prepose.pos.longitude << "," << fname.c_str() << endl;
            
            orignPose = prepose;
            t_R = R.clone();
            t_T = t.clone();
        }
        else
        {
            if((t.at<double>(2) > t.at<double>(0)) &&
               (t.at<double>(2) > t.at<double>(1)))
            {
                t_T = t_T + t_R * t;
                t_R = R * t_R;
            }
            cout << "write " << fname.c_str() << " " << i << endl;
            realfile << curpose.pos.latitude << "," << curpose.pos.longitude << "," << fname.c_str() << endl;
        }
        
//        Mat poseR;
//        Mat poseT;
//        M_Untils::GetRtFromPose(prepose, curpose, cam.RCam2Imu, cam.TCam2Imu, poseR, poseT);
        
        BLHCoordinate _blh;
        M_Untils::CalcPoseFromRT(orignPose, Mat::eye(3, 3, CV_64F), t_T, cam.RCam2Imu, cam.TCam2Imu, _blh);
        
        //计算高斯投影差值
        Point3d realgauss = M_CoorTrans::BLH_to_GaussPrj(curpose.pos);
        Point3d estgauss  = M_CoorTrans::BLH_to_GaussPrj(_blh);
        
        Point3d residual = estgauss - realgauss;
        
         estfile << _blh.latitude << "," << _blh.longitude << "," << residual.x << "," << residual.y << "," << residual.z <<
            "," << fname.c_str() << endl;

        static const int half_size = WIN_SIZE >> 1;
        int x = int(t_T.at<double>(0)) + half_size;
        int y = int(t_T.at<double>(2)) + 400;
        circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);//红色计算轨迹
        rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);

        imshow("display", traj);
        waitKey(1);
        pretime = curtime;
    
        preimg = curimg;
    }
    
    
    getchar();
    realfile.close();
    estfile.close();
    
    return 0;
}
