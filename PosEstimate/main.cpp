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
#include "CFuncHelper.h"
#include "Monocular_CoorTransform.h"
#include "Monocular_Config.h"
#include "Monocular_DataLoader.h"
#include "CFeatureTrack.h"
#include "COptimizer.h"
#include "CFrame.h"
#include "CMap.h"
#include "CViewer.h"


#include "algorithm"

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

const string s_imgRoot = "/media/navinfo/Bak/Datas/@@1002-0001-190828-00/Output/";

const string s_imgPath = s_imgRoot + "LeftCamera/";
const string s_imgGray = s_imgRoot + "gray/";
const string s_excfg   = s_imgRoot + "/extrinsics.xml";
const string s_bscfg   = s_imgRoot + "0001-1220.bs"; 
const string s_pstpath = s_imgRoot + "/1002-0001-190828-00.PosT";

const float s_ImgDis = 1.0;
const int   s_stNo = 640;//2140;
#endif 

const string s_o_rel = "/media/navinfo/Bak/CPP/QT/real.txt";
const string s_o_est = "/media/navinfo/Bak/CPP/QT/est.txt";
const string s_o_img = "/media/navinfo/Bak/CPP/QT/OutImg/";
void inline SwapFrame(Frame *&preframe,Frame *&curframe)
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
     if( inputpath.empty() || 
         outpath.empty() )
         {
             return;
         }
         else
         {
             FileNameVec filenms;
             loadFiles(inputpath,filenms);

             for(size_t i = 0; i < filenms.size() ; ++i)
             {
                 Mat img = imread(filenms[i],CV_LOAD_IMAGE_UNCHANGED);
                 int n = filenms[i].find_last_of('/');
                 std::string outfilename = outpath + filenms[i].substr(++n);
                 
                 Mat grayimg;
                 cvtColor(img,grayimg,CV_RGB2GRAY);
                 printf("%s %d - %d \n", outfilename.c_str(), grayimg.channels(), grayimg.type());
                 imwrite(outfilename,grayimg);
             }
         }
 }

// main func 
int main(int argc, const char * argv[]) {
    // insert code here...

    // cout << cvtwk2dytime(285481.556158) << endl;

    IMURawDataLoader *pLoader = new STIM300IMURawDataLoader();

    pLoader->loadDatas("/media/navinfo/Bak/Datas/@@1002-0001-190828-00/imu/imu190828_071742.txt");
    
    const ImuRawData &data1 = pLoader->getDatas(26467.320);
    const ImuRawData &data2 = pLoader->getDatas(26467.590);
    return 0;

    FrameDrawer *pFdrawer = NULL;
    Map         *pMap     = new Map();
    MapDrawer   *pMpDrawer= new MapDrawer(pMap, "config");
    Viewer      viewer(pFdrawer, pMpDrawer);

    std::shared_ptr<IConfig>  pConfig =
    std::make_shared<WeiYaConfig>(s_excfg,s_bscfg);
    
    Camera cam;
    pConfig->ReadConfig(cam);
    unique_ptr<IDataLoader> pData = std::make_unique<Stim300PostTDataLoader>();

    FileNameVec flnms;
    loadFiles(s_imgGray, flnms);
    pData->loadData(s_pstpath);
    //sort images
    sort(flnms.begin(),flnms.end());

    // Ptr<IFeatureTrack> track = new CVFeatureTrack();

    Ptr<IFeatureTrack> track = new ORBFeatureTrack();

    Frame *preFrame = new Frame;
    Frame *curFrame = NULL;
    CMap    map;

    const int bg_no = s_stNo;
    Mat preimg = imread(flnms[bg_no], CV_LOAD_IMAGE_UNCHANGED);
    
    printf("%d-%d\n",preimg.channels(),preimg.type());
    preFrame->setImage(preimg);

    track->calcFeatures(preimg, preFrame);

    preFrame->initStaticParams();
  
    double pretime = M_Untils::GetDayTimeFromPicName(flnms[bg_no]);
    
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
    
    // map.push(preFrame);
    for(size_t i = bg_no + 1; i < flnms.size() ; ++i)
    {
        size_t n = flnms[i].find_last_of('-');
        const string fname = flnms[i].substr(++n);
        
        Mat curimg = imread(flnms[i],CV_LOAD_IMAGE_UNCHANGED);
        curFrame = new Frame;
        curFrame->setImage(curimg);

        track->calcFeatures(curimg, curFrame);

        Mat matchimg;
        MatchVector mt;

        track->match(preFrame, curFrame,mt);
        
        if( i % 500 == 0 )
        {
            drawMatches(preFrame->getImage(),preFrame->getKeys(),curFrame->getImage(),curFrame->getKeys(),mt,matchimg);

            char matchtxt[255] = {0};
            sprintf(matchtxt, "key count : %d \n matches count : %d",curFrame->getKeys().size(), curFrame->getCurMatchPts().size());
            
            putText(matchimg,matchtxt,Point2i(300,200), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0),3);

            sprintf(matchtxt,"%s/matches_%d.png","/media/navinfo/Bak",i);

            imwrite( matchtxt ,matchimg );
        }
        

        if( curFrame->getCurMatchPts().size() < 10 )
        {
            printf("there has no rigt data for next step...  %d \n", i);
            preFrame = curFrame;
            curFrame = NULL;
            continue;
        }

        Mat E, R, t, mask;
        // E = findEssentialMat( curFrame->getCurMatchPts(),preFrame->getPreMatchPts(), cam.K,RANSAC,
        //                      0.999, 1.0, mask);
        // recoverPose(E,  curFrame->getCurMatchPts(), preFrame->getPreMatchPts(),cam.K,R, t, mask);
        
        E = findEssentialMat( preFrame->getPreMatchPts(),curFrame->getCurMatchPts(), cam.K,RANSAC,
                             0.999, 1.0, mask);
        recoverPose(E, preFrame->getPreMatchPts(), curFrame->getCurMatchPts(), cam.K,R, t, mask);

        double ln = t.at<double>(0,0) * t.at<double>(0,0) +
        t.at<double>(1,0) * t.at<double>(1,0) +
        t.at<double>(2,0) * t.at<double>(2,0);
        ln = s_ImgDis / sqrt(ln);

        t = t * ln;
        
        PoseData prepose = pData->getData(pretime);
        double curtime =  M_Untils::GetDayTimeFromPicName(flnms[i]);
        PoseData curpose = pData->getData(curtime);

       
        Mat result_t ;
        if( bg_no + 1 == i)
        {
            writeRealTrace(realfile, prepose.pos,fname);
            orignPose = prepose;
            t_R = R.clone();
            t_T = t.clone();

            result_t = -R.inv() * t_T; 
        }
        else
        {
            // if((t.at<double>(2) > t.at<double>(0)) &&
            //    (t.at<double>(2) > t.at<double>(1)))
            {
                // t_T = t_T + t_R * t;
                // t_R = R * t_R;

                t_R = R * t_R;
                t_T = R * t_T + t;
                result_t = -t_R.inv() * t_T;
            }
            cout << "write " << fname.c_str() << " " << i << endl;
            writeRealTrace(realfile, prepose.pos,fname);
        }


        const PtVector &curpts = curFrame->getCurMatchPts();

        char ottxt[255] = {0};

        // Mat displayCur = curimg.clone();
        // sprintf(ottxt, "key count : %d \n matches count : %d",curFrame->getKeys().size(), curFrame->getCurMatchPts().size());

        // putText(displayCur,ottxt,Point2i(300,200), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0),3);

        // //draw key points
        // for(size_t j = 0; j < curFrame->getKeys().size();++j)
        // {
        //     circle(displayCur,curFrame->getKeys()[j].pt,6,CV_RGB(0,0,200),3,LINE_AA);
        // }

        // //draw match points
        // for(size_t i = 0 ; i < curpts.size(); ++i)
        // {
        //     circle(displayCur,curpts[i],6,CV_RGB(0,255,0),3,LINE_AA);
        //     // const int r = 8;
        //     // Rect2i rect(curpts[i].x,curpts[i].y,r,r);
        //     // rectangle(displayCur,rect,CV_RGB(0,255,0));
        // }

        preFrame = curFrame;
        map.push(curFrame);
        
        curFrame = NULL;
        
//        Mat poseR;
//        Mat poseT;
//        M_Untils::GetRtFromPose(prepose, curpose, cam.RCam2Imu, cam.TCam2Imu, poseR, poseT);
        
        BLHCoordinate _blh;
        // M_Untils::CalcPoseFromRT(orignPose, Mat::eye(3, 3, CV_64F), t_T, cam.RCam2Imu, cam.TCam2Imu, _blh);
         M_Untils::CalcPoseFromRT(orignPose, Mat::eye(3, 3, CV_64F), result_t, cam.RCam2Imu, cam.TCam2Imu, _blh);
        //计算高斯投影差值
        Point3d realgauss = M_CoorTrans::BLH_to_GaussPrj(curpose.pos);
        Point3d estgauss  = M_CoorTrans::BLH_to_GaussPrj(_blh);
        
        Point3d residual = estgauss - realgauss;
  
        writeEstTrace(estfile,_blh,residual,fname);

        // static const int half_size = WIN_SIZE >> 1;
        // int x = int(t_T.at<double>(0)) + half_size;
        // int y = int(t_T.at<double>(2)) + 400;
        // static const int half_size = WIN_SIZE >> 1;
        // int x = int(result_t.at<double>(0)) + half_size;
        // int y = int(result_t.at<double>(2)) + 400;
        // circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);//红色计算轨迹
        // rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);

        // Mat imgdisplay;

        // resize(curimg,imgdisplay,Size( (WIN_SIZE * 1.5), (WIN_SIZE ) ));

        // imshow("img",imgdisplay);
        // imshow("display", traj);
        waitKey(1);

        //system("free -h");
         Mat t_t = Mat::eye(4, 4, CV_64F);
        t_R.copyTo(t_t.rowRange(0,3).colRange(0, 3));//

        t_T.copyTo( t_t.rowRange(0, 3).col(3));
        pMpDrawer->SetCurrentCameraPose(t_t);
        
        KeyFrame *kframe = new KeyFrame();
        kframe->SetWorldPos(t_t);
        pMap->push(kframe);

        viewer.Run();

        pretime = curtime;
    }
    
    getchar();
    realfile.close();
    estfile.close();
    
    return 0;
}
