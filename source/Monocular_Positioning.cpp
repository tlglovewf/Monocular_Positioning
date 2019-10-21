//
//  Monocular_Positioning.cpp
//  MonocularPositioning
//
//  Created by TuLigen on 2019/6/4.
//  Copyright © 2019年 TuLigen. All rights reserved.
//

#include "Monocular_Positioning.h"
#include "Monocular_Utils.h"
#include "Monocular_CoorTransform.h"
#include "Monocular_DataLoader.h"
#include "Monocular_BlockMatching.h"

#define MAINMETHOD          1
#define EPILINESEARCHLEN    800  //极线搜索距离
#define ALLOWSCORE          0.90 //块匹配 评分阀值

Monocular_Positioning::Monocular_Positioning(const string &outpath )
{
    mpDataLoader = std::make_unique<Stim300PostTDataLoader>();
    mpSer        = std::make_unique<FileSerialization>(outpath);
    mOutPutDir   = outpath;
}

Monocular_Positioning::~Monocular_Positioning()
{
}
/*
 * 加载imu数据
 * @param path  数据路径
 */
void Monocular_Positioning::LoadImuData(const string &path)
{
    mpDataLoader->loadData(path);
}
/*
 * 根据图片名称获取imu数据
 * @param  picname 图片名称
 * @return         姿态数据
 */
PoseData Monocular_Positioning::GetItemData(const std::string &picname)
{
    double t = M_Untils::GetDayTimeFromPicName(picname);
    PoseData posedata = mpDataLoader->getData(t);
    return posedata;
}
/*
 * 从R 和 t 中恢复 极线
 * @param R       帧间旋转矩阵
 * @param t       帧间平移矩阵
 * @param pixel   像素值
 * @param a,b,c   极线 ax + by + c = 0;
 */
void  Monocular_Positioning::GetEpilineFromPose(
                                                const Mat &R, const Mat &t,
                                                const Point2f &pixel,
                                                double &a, double &b, double &c)
{
	cout << "Get Epiline!!!" << endl;
	//cout << "R : " << R << endl;
	//cout << "t : " << t << endl;
    //计算本质矩阵
    Mat E = M_Untils::ComputeEssentialMat(R,t);
	//cout << "E : " << E << endl;
    //计算基础矩阵
    Mat F = M_Untils::ComputeFundamentalMat(E, mCamera.K, mCamera.K);
	//cout << "F : " << F << endl;
    //计算极线
    M_Untils::CreateEpiline(F, Point2f(pixel.x, pixel.y), a, b, c);
	cout << "End Epiline!!!" << endl;
}


/*
 * 极线匹配
 * @param a,b,c     极线
 * @param targets   目标列表
 * @param box       待匹配目标
 */
Target Monocular_Positioning::EpilineMatch(double a, double b, double c, const TargetVector &targets, const Target &item)
{
	cout << "Begin target matching." << endl;
    Point2f bg = item.center();
    Point2f ed(bg.x + EPILINESEARCHLEN,0);
    ed.y = M_Untils::ComputeY(ed.x, a, b, c);
    
    bg.x -= EPILINESEARCHLEN;
    bg.y =  M_Untils::ComputeY(bg.x, a, b, c);
    
    
    for( Target target : targets)
    {
        if( !BLHCoordinate::isValid(target._pos) &&
             (target._type == item._type) )
        {//检测到类型相同且为赋值 才进行下一步判断
            if(M_Untils::IsIntersect(bg, ed, target._box ))
            {//相交则认为是同一个物体,并不再进行下一步判断
				cout << "Match a correct target.." << endl;
                return target;
            }
        }
    }
	cout << "NO same target." << endl;
    return Target();
}

/*
 * 极线匹配(基于块匹配)
 * @param a,b,c     极线
 * @param preframe  前帧
 * @param curframe  后帧
 */
Point2f Monocular_Positioning::EpilineMatch(double a, double b, double c,
                     const FrameData &preframe,
                     const FrameData &curframe,
                     const Point2f   &pt,
                     Mat &img)
{
    //左移 还是右移动  基于寻找同名点的x值 与 1/2 图像大小进行判断
    int  moveleft = (pt.x > (preframe._img.cols >>1) ) ? 1 : -1;
    Point2f bg = pt;
    Point2f ed(bg.x,0);
    ed.y = M_Untils::ComputeY(ed.x, a, b, c);
    
    float best_score    = -1.0;
    Point2f best_px_curr ;
    const int space = 1;
    const int searchlen = moveleft * EPILINESEARCHLEN  + bg.x;
    
    int st_x = bg.x;
    int ed_x = searchlen;
    
    if(moveleft < 0)
    {
        st_x = searchlen;
        ed_x = bg.x;
    }
	
    std::auto_ptr<BlockMatch> pBlock(BlockMatch::CreateMethod(eNCC, preframe._img, pt));
    Time_Interval timer;
    timer.start();
    for(int i = st_x; i < ed_x; i += space)
    {
        Point2f px_curr(i, M_Untils::ComputeY(i, a,b,c));
        
        double dscore =  pBlock->score(curframe._img, px_curr);
        
        if(dscore > best_score)
        {
            best_score = dscore;
            best_px_curr = px_curr;
        }
#if OUTPUTRESULT
		circle(img, px_curr, 3, CV_RGB(255,0,0));//绘制匹配线
#endif
    }
    
    cout << "The Best score is " << best_score << "!!!!" << endl;

	timer.prompt("BlockMatching cost ");

    if(best_score > ALLOWSCORE)
    {

#if OUTPUTRESULT
		char tmp[64] = { 0 };

		sprintf(tmp, "%f", best_score);
		
		putText(img, tmp, best_px_curr, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 3);
#endif

        return best_px_curr;
    }
    else
    {
		return Point2f(WRONGDATA, 0);
    }
}


/*
 * 绘制极线
 * @param img   图片
 * @param path  输出路径
 * @param pt    图像点
 * @param a,b,c 极线
 *
 */
void Monocular_Positioning::OutputEpiline(Mat &img,
                   const Target &pre, const Target &cur,
                   double a, double b, double c)
{
    if(img.empty())
        return;
	//rectangle(img, pre._box.tl(), pre._box.br(), CV_RGB(255,100,0),2);//preframe target
	rectangle(img, cur._box.tl(), cur._box.br(), CV_RGB(0,255,0),2);//curframe target
    OutputEpiline(img, pre.center(), cur.center(), a, b, c);
}

void Monocular_Positioning::OutputEpiline(Mat &img,
                   const Point2f &prept,const Point2f &curpt,
                   double a, double b, double c)
{
    if(img.empty())
        return;
    
	//circle(img, prept, 5, CV_RGB(255, 100, 0), 2);
	circle(img, curpt, 5, CV_RGB(0, 255, 0), 2);
}

/*
 * 定位
 * @param prept     图片
 * @param curpt     输出路径
 * @param predata   前帧imu数据
 * @param curdata   当前帧imu数据
 * @param R         旋转矩阵
 * @param t         平移矩阵
 * @return          坐标
 */
BLHCoordinate Monocular_Positioning::Position(const Point2f &prept,
                                     const Point2f &curpt,
                                     const PoseData &predata,
                                     const PoseData &curdata,
                                     Mat &R, Mat &t)
{
    const BLHCoordinate &blht1  = predata.pos;
    const BLHCoordinate &blht2  = curdata.pos;
    
    //计算imu到enu 转换矩阵
	cv::Mat Rimu2Enu1 = M_CoorTrans::IMU_to_ENU(-predata._yaw, predata._pitch, predata._roll);
	cv::Mat Rimu2Enu2 = M_CoorTrans::IMU_to_ENU(-curdata._yaw, curdata._pitch, curdata._roll);
    
    //计算xyz转到enu 转换矩阵
	cv::Mat XYZ2Enu1 = M_CoorTrans::XYZ_to_ENU(blht1.latitude, blht1.longitude);
	cv::Mat XYZ2Enu2 = M_CoorTrans::XYZ_to_ENU(blht2.latitude, blht2.longitude);
    
    //imu到 xyz转换矩阵
    cv::Mat Rimu2xyzt1 = XYZ2Enu1.t() * Rimu2Enu1;
    cv::Mat Rimu2xyzt2 = XYZ2Enu2.t() * Rimu2Enu2;
    
    
    Point3d xyzt1;
    Point3d xyzt2;
    //获取xyz坐标
	xyzt1 = M_CoorTrans::BLH_to_XYZ(blht1);
	xyzt2 = M_CoorTrans::BLH_to_XYZ(blht2);
    cv::Mat pt1 = (cv::Mat_<double>(3, 1) << xyzt1.x, xyzt1.y, xyzt1.z);
    cv::Mat pt2 = (cv::Mat_<double>(3, 1) << xyzt2.x, xyzt2.y, xyzt2.z);
    
    //相对旋转矩阵
#if MAINMETHOD
    
    R = mCamera.RCam2Imu.t() * Rimu2xyzt2.t() * Rimu2xyzt1 * mCamera.RCam2Imu;//R
    
    //计算 cur相机xyz坐标
    cv::Mat pcamt2 = Rimu2xyzt2 * mCamera.TCam2Imu + pt2;
    //计算 cur相机在pre imu坐标系位置
    cv::Mat imut1Pcam = Rimu2xyzt1.t()*pcamt2 - Rimu2xyzt1.t()*pt1;
    //计算 cur 到 pre相机 的t
    cv::Mat camt1Pcam = mCamera.RCam2Imu.t() * imut1Pcam - mCamera.RCam2Imu.t() * mCamera.TCam2Imu;
    //根据R 计算具体t
    t = -R * camt1Pcam;//t
    
#else
    // △R
    R =  Rimu2xyzt2 * Rimu2xyzt1.t() ;
    // △t -> imu -> cam -> R
    t =  mCamera.RCam2Imu.inv() *  Rimu2xyzt1.inv() * ( pt1 - pt2 );
    
#endif
    
    Point2d dp1 = M_Untils::Pixel2Cam(prept, mCamera.K);
    Point2d dp2 = M_Untils::Pixel2Cam(curpt, mCamera.K);
    
    std::vector<Point2d> pts_1, pts_2;
    
    pts_1.push_back(dp1);
    pts_2.push_back(dp2);
    
    Mat T1 =  Mat::eye(3, 4, CV_64F);
    
    Mat T2;
    hconcat(R, t, T2);
    
    Mat pt_4d;
    //三角测量
    triangulatePoints(T1, T2,pts_1, pts_2, pt_4d);
    
    Mat x = pt_4d.col(0);
    
    x = x/x.at<double>(3,0);
    
    //合并cam -> imu r + t 转换矩阵
    Mat cam2imu(4,4,CV_32FC1);
    mCamera.RCam2Imu.copyTo(cam2imu);
    hconcat(mCamera.RCam2Imu, mCamera.TCam2Imu, cam2imu);
    
    //cam -> imu -> enu -> xyz
    Mat mrst =  pt1 + XYZ2Enu1.t() * Rimu2Enu1 *  cam2imu * x;
    
    Point3d xyz;
    xyz.x = mrst.at<double>(0,0);
    xyz.y = mrst.at<double>(1,0);
    xyz.z = mrst.at<double>(2,0);
    
	return M_CoorTrans::XYZ_to_BLH(xyz);
}

/*
 * 定位
 * @param preframe  前帧
 * @param curframe  后帧
 * @param results   结果
 */
bool Monocular_Positioning::Position(FrameData    &preframe,
                                     FrameData    &curframe,
                                     ResultVector &results)
{
	if (
		preframe._targets.empty()||
		preframe._img.empty() ||
		curframe._img.empty() ||
		preframe._pose._t < 0 ||
		curframe._pose._t < 0   
	   )
    {
		cout << "Targetlist is empty,or pose(img) data error! Position failed. " << endl;
        return false;
    }
	cout << "Position Start!!!" << endl;
    const BLHCoordinate &blht1  = preframe._pose.pos;
    const BLHCoordinate &blht2  = curframe._pose.pos;

    const PoseData &predata = preframe._pose;
    const PoseData &curdata = curframe._pose;
    
    //计算imu到enu 转换矩阵
	cv::Mat Rimu2Enu1 = M_CoorTrans::IMU_to_ENU(-predata._yaw, predata._pitch, predata._roll);
	cv::Mat Rimu2Enu2 = M_CoorTrans::IMU_to_ENU(-curdata._yaw, curdata._pitch, curdata._roll);

    //计算xyz转到enu 转换矩阵
	cv::Mat XYZ2Enu1 = M_CoorTrans::XYZ_to_ENU(blht1.latitude, blht1.longitude);
	cv::Mat XYZ2Enu2 = M_CoorTrans::XYZ_to_ENU(blht2.latitude, blht2.longitude);

    //imu到 xyz转换矩阵
    cv::Mat Rimu2xyzt1 = XYZ2Enu1.t() * Rimu2Enu1;
    cv::Mat Rimu2xyzt2 = XYZ2Enu2.t() * Rimu2Enu2;

    Point3d xyzt1;
    Point3d xyzt2;
    //获取xyz坐标
	xyzt1 = M_CoorTrans::BLH_to_XYZ(blht1);
	xyzt2 = M_CoorTrans::BLH_to_XYZ(blht2);
    cv::Mat pt1 = (cv::Mat_<double>(3, 1) << xyzt1.x, xyzt1.y, xyzt1.z);
    cv::Mat pt2 = (cv::Mat_<double>(3, 1) << xyzt2.x, xyzt2.y, xyzt2.z);

    Mat R;
    Mat t;
    //相对旋转矩阵
#if MAINMETHOD

    R = mCamera.RCam2Imu.t() * Rimu2xyzt2.t() * Rimu2xyzt1 * mCamera.RCam2Imu;//R

    //计算 cur相机xyz坐标
    cv::Mat pcamt2 = Rimu2xyzt2 * mCamera.TCam2Imu + pt2;
    //计算 cur相机在pre imu坐标系位置
    cv::Mat imut1Pcam = Rimu2xyzt1.t()*pcamt2 - Rimu2xyzt1.t()*pt1;
    //计算 cur 到 pre相机 的t
    cv::Mat camt1Pcam = mCamera.RCam2Imu.t() * imut1Pcam - mCamera.RCam2Imu.t() * mCamera.TCam2Imu;
    //根据R 计算具体t
    t = -R * camt1Pcam;//t

#else
    // △R
    R =  Rimu2xyzt2.t() * Rimu2xyzt1 ;
    // △t -> imu -> cam -> R
    t =  mCamera.RCam2Imu.inv() *  Rimu2xyzt1.inv() * ( pt1 - pt2 );
    
  
#endif

    //合并cam -> imu r + t 转换矩阵
    Mat cam2imu;
    mCamera.RCam2Imu.copyTo(cam2imu);
    hconcat(mCamera.RCam2Imu, mCamera.TCam2Imu, cam2imu);
    
#if OUTPUTRESULT
    Mat resultimg = curframe._img.clone();

	char tmp[255] = { 0 };

	mpSer->prompt("======================");
	mpSer->prompt(curframe._name);
    mpSer->writeFormat("frame gps", curframe._pose.pos);
	//mpSer->writeFormat("R", R);
	//mpSer->writeFormat("t", t);
#endif
	cout << "calc R and t successfully." << endl;
    for(auto &target : preframe._targets)
    {
        //目标已有值,则不再计算 直接返回
        if(BLHCoordinate::isValid(target._pos))
        {
            continue;
        }
        else
        {
            const Point2f tgcenter = target.center();
            Point2d dp1 = M_Untils::Pixel2Cam(tgcenter, mCamera.K);
            double a,b,c;
            
            //获取极线
            GetEpilineFromPose(R, t, tgcenter, a, b, c);

#if OUTPUTRESULT
			Point2d bg;
			bg.x = 0;
			bg.y = M_Untils::ComputeY(bg.x, a, b, c);
			Point2d ed;
			ed.x = resultimg.cols;
			ed.y = M_Untils::ComputeY(ed.x, a, b, c);
			line(resultimg, bg, ed, CV_RGB(0, 255, 255), 2);//draw epiline
#endif

            //极线匹配
			Target matchingtg = EpilineMatch(a, b, c, curframe._targets, target);
            
            Point2f smpt;
			if (!Target::isValid(matchingtg))
			{//基于目标包围盒没有匹配到 则进入块匹配
				cout << "Block Matching start." << endl;
#if OUTPUTRESULT
				smpt = EpilineMatch(a, b, c, preframe, curframe, target.center(), resultimg);
#else
				Mat m;
				smpt = EpilineMatch(a, b, c, preframe, curframe, target.center(),m);
#endif

#if OUTPUTRESULT
				OutputEpiline(resultimg, target.center(), smpt, a, b, c);
#endif
				cout << "Block Matching end." << endl;
                if(smpt.x < 0)
                {//块匹配失败 则返回
					cout << target._type << " no same point. position failed!!" << endl;

					continue;
                }
            }
            else
            {
                smpt =  matchingtg.center();

#if OUTPUTRESULT
				OutputEpiline(resultimg, target, matchingtg, a, b, c);
#endif
            }
            

           
            std::cout << "The matching point is : " << smpt << std::endl;
            Point2d dp2 =
            M_Untils::Pixel2Cam(smpt, mCamera.K);
            
            std::vector<Point2d> pts_1, pts_2;
            
            pts_1.push_back(dp1);
            pts_2.push_back(dp2);
            
            Mat T1 =  Mat::eye(3, 4, CV_64F);
            
            Mat T2;
            hconcat(R, t, T2);
            
            Mat pt_4d;
            //三角测量
            triangulatePoints(T1, T2,pts_1, pts_2, pt_4d);
            
            Mat x = pt_4d.col(0);
            
            x = x / x.at<double>(3,0);
           
            //cam -> imu -> enu -> xyz
            Mat mrst =  pt1 + XYZ2Enu1.t() * Rimu2Enu1 * cam2imu * x;
           
            Point3d xyz;
            xyz.x = mrst.at<double>(0,0);
            xyz.y = mrst.at<double>(1,0);
            xyz.z = mrst.at<double>(2,0);
            
            Result result;
            result._type = 1;
			result._pos = M_CoorTrans::XYZ_to_BLH(xyz);
            
            results.emplace_back(result);
            
            target._pos = result._pos;
            
#if OUTPUTRESULT
			mpSer->prompt("----------");
			mpSer->writeFormat("type "  , target._type);
			mpSer->writeFormat("prept"  , target.center());
			mpSer->writeFormat("curpt"  , smpt);
            mpSer->writeFormat("geopt"	, target._pos);
			Point3d gausspt = M_CoorTrans::BLH_to_GaussPrj(target._pos);
			mpSer->writeFormat("gauss"	, gausspt);
			mpSer->space();
#endif
        }
    }

#if OUTPUTRESULT
	sprintf(tmp, "%s/%s_result.jpg", mOutPutDir.c_str(), M_Untils::GetNameNo(curframe._name).c_str());

	cout << "Save matching image at " << tmp << endl;
	char outtext[1024] = { 0 };

	sprintf(outtext, "daytime : %lf", curframe._pose._t);

	SaveImage(tmp, resultimg, outtext);
#endif

	cout << "Position End!!!" << endl;
    return true;
}


/*
* 存储图片
* @param path  存储路径
* @param img   需要存储的图片
* @param promt 文字
*/
void Monocular_Positioning::SaveImage(const std::string &path, Mat &img, const std::string &promt /*= ""*/)
{
	if (!path.empty() &&  !img.empty())
	{
		if (!promt.empty())
		{
			float x = img.cols / 3;
			float y = 100;
			putText(img, promt, Point2f(x, y), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 3);
		}
		imwrite(path, img);
	}
}


/*
 * 反投
 */
Point2f Monocular_Positioning::BackProject(const FrameData &frame,const BLHCoordinate &blh, Mat &outimg)
{
    //calc target xyz coordinate
	Point3d  xyz = M_CoorTrans::BLH_to_XYZ(blh);
    
    //calc cam xyz coordinate
	Point3d  cam = M_CoorTrans::BLH_to_XYZ(frame._pose.pos);
    
    Point3d dt = xyz - cam;
    
    //trans pt -> mat
    Mat tmp = (Mat_<double>(3,1) << dt.x ,dt.y,dt.z);
    
    //xyz -> enu
	cv::Mat XYZ2Enu1 = M_CoorTrans::XYZ_to_ENU(frame._pose.pos.latitude, frame._pose.pos.longitude);
    
    //计算imu到enu 转换矩阵
	cv::Mat Rimu2Enu1 = M_CoorTrans::IMU_to_ENU(-frame._pose._yaw, frame._pose._pitch, frame._pose._roll);
    
    //xyz->enu->imu
    Mat rst = Rimu2Enu1.t() * XYZ2Enu1 * tmp;
    
    //imu-> cam
    rst = rst - mCamera.TCam2Imu;
    rst =  mCamera.RCam2Imu.t() *  rst ;
    
    Mat t = mCamera.K * rst ;
    
    Point2f pt;
    pt.x = t.at<double>(0,0) / t.at<double>(2,0);
    pt.y = t.at<double>(1,0) / t.at<double>(2,0);

	circle(outimg, pt, 4, CV_RGB(0, 255, 0), LINE_AA);
    
    return pt;
}

bool Monocular_Positioning::SetConfig(const std::shared_ptr<IConfig> &config)
{
	return config->ReadConfig(mCamera);
}



#pragma region test

double GetDistance(BLHCoordinate pre, BLHCoordinate cur)
{
	Point3d xyz1 = M_CoorTrans::BLH_to_XYZ(pre);
	Point3d xyz2 = M_CoorTrans::BLH_to_XYZ(cur);
	Point3d rst{ xyz2.x - xyz1.x, xyz2.y - xyz1.y, xyz2.z - xyz1.z };
	return sqrt(rst.x * rst.x + rst.y * rst.y + rst.z * rst.z);
}

static inline Rect2f GetBoxFromCenter(const Point2f &pt)
{
	const int len = 50;
	const int doublelen = len << 1;
	return Rect2f(pt.x - len, pt.y - len, doublelen, doublelen);
}


static inline std::string GetFileName(const std::string &filepath)
{
	int index1 = filepath.find_last_of('/');
	int index2 = filepath.find_last_of('\\');

	int index = max(index1, index2);//取两个符号中最大的位名称字符起始位置
	assert(index > 0);
	return filepath.substr(index + 1);
}


/*
 *   测试
 */
void Monocular_Positioning::Test()
{
	FrameData preframe;
	FrameData curframe;

	Target preitem;
	preitem._type = 50;
	preitem._box = GetBoxFromCenter(Point2f(2604, 401));
	preframe._targets.push_back(preitem);
	
	preitem._type = 14501;
	preitem._box = GetBoxFromCenter(Point2f(2823, 392));
	preframe._targets.push_back(preitem);
	
	preitem._type = 14301;
	preitem._box = GetBoxFromCenter(Point2f(2712, 396));
	preframe._targets.push_back(preitem);
	
    
    preitem._type = 44;
    preitem._box = GetBoxFromCenter(Point2f(2043,527));
    preframe._targets.push_back(preitem);
    
    preitem._type = 45;
    preitem._box = GetBoxFromCenter(Point2f(2610,1180));
    preframe._targets.push_back(preitem);
    
    preitem._type = 46;
    preitem._box = GetBoxFromCenter(Point2f(2807,178));
    preframe._targets.push_back(preitem);
    
	Target curitem;
	curitem._type = 50;
	curitem._box = GetBoxFromCenter(Point2f(2445, 308));
	curframe._targets.push_back(curitem);
	
	curitem._type = 14501;
	curitem._box = GetBoxFromCenter(Point2f(2698, 301));
	curframe._targets.push_back(curitem);
	
	curitem._type = 14301;
	curitem._box = GetBoxFromCenter(Point2f(2570, 303));
	curframe._targets.push_back(curitem);;
    
    curitem._type = 44;
    curitem._box = GetBoxFromCenter(Point2f(1793,487));
    curframe._targets.push_back(curitem);
    
    curitem._type = 45;
    curitem._box = GetBoxFromCenter(Point2f(2575,1321));
    curframe._targets.push_back(curitem);
    
    curitem._type = 46;
    curitem._box = GetBoxFromCenter(Point2f(2659,69));
    curframe._targets.push_back(curitem);
    
	BLHCoordinate truth1{ 30.44622693, 114.47132012, 26.5568 };
	BLHCoordinate truth2{ 30.44623611, 114.47132058, 26.5421 };
	BLHCoordinate truth3{ 30.44624632, 114.47132203, 26.4959 };
	BLHCoordinate truths[] = { truth1, truth2, truth3 };


	std::shared_ptr<IConfig>  pConfig =
		std::make_shared<WeiYaConfig>(mOutPutDir + "/../data/EX_0003_20190614.xml",
		mOutPutDir + "/../data/BS_0003_20190614.bs");

	LoadImuData(mOutPutDir + "/../data/20190615.PosT");

	SetConfig(pConfig);

	const std::string prestr = mOutPutDir + "/../data/20190615-065016867685-0000000100_L.jpg";
	const std::string curstr = mOutPutDir + "/../data/20190615-065017311595-0000000101_L.jpg";

	preframe._name = GetFileName(prestr);
	curframe._name = GetFileName(curstr);

	preframe._img = imread(prestr);
	curframe._img = imread(curstr);

	cout.precision(15);
	preframe._pose =GetItemData(preframe._name);
	curframe._pose =GetItemData(curframe._name);

	ResultVector results;

	if (!Position(preframe, curframe, results))
	{
		cout << "failed !!! " << endl;
	}

	Mat outimg = preframe._img.clone();
	for (auto item : results)
	{
		BackProject(preframe, item._pos, outimg);
	}

	imwrite(mOutPutDir + "/backproject.jpg", outimg);

	int index = 0;
	for (auto t : preframe._targets)
	{
		cout << "+++++++++++++++++++++++++" << endl;

		cout << "e : " << t._pos.latitude << "," << t._pos.longitude << ","
			<< t._pos.altitude << endl;
		cout << "t : " << truths[index].latitude << "," << truths[index].longitude << "," << truths[index].altitude << endl;
		cout << "d : " << GetDistance(t._pos, truths[index++]) << endl;

		cout << "-------------------------" << endl;
	}
}


#pragma endregion
