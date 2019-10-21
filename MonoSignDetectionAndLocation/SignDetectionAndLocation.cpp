#include "SignAdapterClassify.h"
#include "SignAdapterDetectSSD.h"
#include "SignAdapterWebServiceRequest.h"
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <chrono>
#include <stdlib.h>

#include "MMSPOS.h"
#include "MMSStereoPointCloud.h"

#include "SignDetectionAndLocation.h"
#include <iostream>

#ifdef WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

#include "ConsoleFunction.h"

#include "SignAdapterTracker.h"

using namespace std;
using namespace SignSDK;
using namespace ConsoleSpace;
namespace fs = boost::filesystem;

namespace FunctionSpace
{
	SSDDetect::SSDDetect(int argc, char** argv)
	{
		iniFlag = Init(argc, argv);
	}

	bool SSDDetect::Init(int argc, char** argv)
	{
		std::string root = "";
#ifdef WIN32
		char *buffer = _getcwd(NULL, 0);
		if (buffer != NULL)
		{
			root = buffer;
			free(buffer);
		}
#else
		char buffer[1024];
		if (getcwd(buffer, 1024) != NULL)
		{
			root = buffer;
		}
#endif

		m_input = "";
		m_output = "";
		m_display = Hidden;
		m_gpuID = 0;

		std::string funcName = "";

		//parse arugs
		for (int i = 0; i < argc; i++)
		{
			if (!strcmp(argv[i], "-func"))
				funcName = argv[++i];
			else if (!strcmp(argv[i], "-i"))
				m_input = argv[++i];
			else if (!strcmp(argv[i], "-o"))
				m_output = argv[++i];
			else if (!strcmp(argv[i], "-v"))
				m_display = ShowImage;
			else if (!strcmp(argv[i], "-s"))
				m_start = atoi(argv[++i]);
			else if (!strcmp(argv[i], "-e"))
				m_end = atoi(argv[++i]);
			else if (!strcmp(argv[i], "-pos"))
				m_posfile = argv[++i];
			else if (!strcmp(argv[i], "-GPU"))
			{
				std::stringstream ss;
				ss << argv[++i];
				ss >> m_gpuID;
			}
		}

		if (!fs::exists(m_output))
		{
			if (!fs::create_directory(m_output))
			{
				std::cout << "Failed to create dir " << m_output << std::endl;
				return false;
			}
		}

		m_ptrContainer = std::make_shared<SignAdapterContainer>();
		m_ptrContainer->AddAdapter(new SignAdapterDetectSSD());
		m_ptrContainer->AddAdapter(new SignAdapterClassify());
		m_ptrContainer->AddAdapter(new SignAdapterTracker());

		SignAdapterDetectSSD* detectAdapter = m_ptrContainer->GetAdapter<SignAdapterDetectSSD>(adapter_detect_ssd);
		std::stringstream ssdModelSS;
		ssdModelSS << root << "/models/SSDmodel/model.caffemodel";
		std::string ssdModelFile = ssdModelSS.str();
		if (!detectAdapter->InitialDetector(ssdModelFile, m_gpuID))
		{
			return false;
		}

		SignAdapterClassify* classifyAdapter = m_ptrContainer->GetAdapter<SignAdapterClassify>(adapter_classify);
		std::string mixModelDir = root + "/models";
		if (!classifyAdapter->InitialMixClassify(mixModelDir, m_gpuID))
		{
			return false;
		}


	//通过解算数据单目定位
	/*以下是一些计算函数，在上次整理的代码中有，只需整合一下*/
#define D2R(X)   (X) * 0.01745329251994329547437168059786927187815308570862
#define R2D(X)   (X) / 0.01745329251994329547437168059786927187815308570862

	const double PI64 = 3.1415926535897932384626433832795028841971693993751;//Pi

	struct BLHCoordinate
	{
		double latitude;

		double longitude;

		double altitude;
	};

	struct XYZCoordinate
	{
		double x;

		double y;

		double z;
	};

	struct Datum
	{
		double r_max;
		double r_min;
		double e2;
	};

	static  Datum WGS84Datum{ 6378137, 6356752.3142, 0.00669437999013 };

	static XYZCoordinate BLH_to_XYZ(const BLHCoordinate &blh, Datum dt = WGS84Datum)
	{
		double a = dt.r_max;
		double e = dt.e2;//sqrt(a * a - b * b) / a;
		double N = a / sqrt(1 - e  * sin(D2R(blh.latitude)) * sin(D2R(blh.latitude)));
		double WGS84_X = (N + blh.altitude) * cos(D2R(blh.latitude)) * cos(D2R(blh.longitude));
		double WGS84_Y = (N + blh.altitude) * cos(D2R(blh.latitude)) * sin(D2R(blh.longitude));
		double WGS84_Z = (N * (1 - e) + blh.altitude) * sin(D2R(blh.latitude));

		return{ WGS84_X, WGS84_Y, WGS84_Z };
	}

	static BLHCoordinate XYZ_to_BLH(const XYZCoordinate &pt, Datum dt = WGS84Datum)
	{
		double f, f1, f2;
		double p, zw, nnq;
		double b, l, h;

		double a = dt.r_max;
		double eq = dt.e2;
		f = PI64 * 50 / 180;
		double x, y, z;
		x = pt.x;
		y = pt.y;
		z = pt.z;
		p = z / sqrt(x * x + y * y);
		do
		{
			zw = a / sqrt(1 - eq * sin(f) * sin(f));
			nnq = 1 - eq * zw / (sqrt(x * x + y * y) / cos(f));
			f1 = atan(p / nnq);
			f2 = f;
			f = f1;
		} while (!(abs(f2 - f1) < 10E-10));
		b = R2D(f);
		l = R2D(atan(y / x));
		if (l < 0)
			l += 180.0;
		h = sqrt(x * x + y * y) / cos(f1) - a / sqrt(1 - eq * sin(f1) * sin(f1));
		return{ b, l, h };
	}

	cv::Mat CalcRxyz2nav(const double& B, const double& L)
	{
		cv::Mat XYZ2Navi = (cv::Mat_<double>(3, 3) <<
			-sin(D2R(L)), cos(D2R(L)), 0,
			-sin(D2R(B))*cos(D2R(L)), -sin(D2R(L))*sin(D2R(B)), cos(D2R(B)),
			cos(D2R(L))*cos(D2R(B)), cos(D2R(B))*sin(D2R(L)), sin(D2R(B)));
		return XYZ2Navi;
	}

	cv::Mat CalcRimu2nav(const double& yaw, const double& pitch, const double& roll)
	{
		cv::Mat rollR = (cv::Mat_<double>(3, 3) << cos(D2R(roll)), 0, sin(D2R(roll)),
			0, 1, 0,
			-sin(D2R(roll)), 0, cos(D2R(roll)));

		cv::Mat pitchR = (cv::Mat_<double>(3, 3) << 1, 0, 0,
			0, cos(D2R(pitch)), -sin(D2R(pitch)),
			0, sin(D2R(pitch)), cos(D2R(pitch)));

		cv::Mat yawR = (cv::Mat_<double>(3, 3) << cos(D2R(yaw)), -sin(D2R(yaw)), 0,
			sin(D2R(yaw)), cos(D2R(yaw)), 0,
			0, 0, 1);
		return yawR*pitchR*rollR;
	}
/*以上是一些计算函数，在上次整理的代码中有，只需整合一下*/
	struct MySignRes//自定义的标牌结果
	{
		double lat;
		double lon;
		double h;
		int label;
		int seq;
	};

	void SSDDetect::MonoSignDetectAndLocation()//标牌识别&定位
	{
		if (!iniFlag)
		{
			std::cout << "Failed to init" << std::endl;

			return;
		}

		/*读取的bs文件参数*/
		Mat Rcam2imu = (Mat_<double>(3, 3) << 9.9992050579194636e-01, -1.1412508130255723e-02,
			-5.3606673982689035e-03, 5.3590667889582789e-03,
			-1.7082723484040588e-04, 9.9998562550729064e-01,
			-1.1413259829229354e-02, -9.9993486061654668e-01,
			-1.0965326176508848e-04);
		Mat Tcam2imu = (Mat_<double>(3, 1) << -6.8060955133028156e-01, 5.1348671129742951e-02,
			1.1180491927652475e-01);
		/*读取的相机内参*/	
		Mat K = (Mat_<double>(3, 3) <<
			1.7842962813417348e+03, 0., 2.0476912918090820e+03,
			0., 1.7842962813417348e+03, 1.0898443069458008e+03,
			0., 0., 1.);
			
		/*weiyaSDK获取图片名与轨迹的对应*/
		MMSPOS imagePosT;
		imagePosT.setPOSFileName(m_posfile);

		if (!imagePosT.loadPOSData())
		{
			std::cout << "posT file failed!" << std::endl;
		}

		std::vector<ISignDetector::Sign> detectionslast;//前一帧识别结果
		detectionslast.clear();
		string imagelast = "";
		
		fs::directory_iterator end_itr;
		for (fs::directory_iterator itr(m_input);
			itr != end_itr;
			++itr){
			
			if (fs::is_regular_file(itr->status())){
				string curImgName = itr->path().string();
				cv::Mat image = cv::imread(curImgName);
				std::cout << "current img:" << curImgName << std::endl;
				auto index = itr->path().string().find_last_of("-") + 6;
				//auto strlen = itr->path().string().find_last_of("_") - index;
				int imgNum = atoi(itr->path().string().substr(index, 5).c_str());
				if (imgNum / 10000 == 0)
					imgNum = imgNum % 10000;
				//cout << "imgNum:" << imgNum << endl;
				if (image.empty() || !image.data)
					continue;
				resize(image, image, Size(image.cols / 2, image.rows / 2));
				//cout << image.rows << image.cols << endl;
				std::vector<ISignDetector::Sign> detections;
				SignAdapterDetectSSD* detectAdapter = m_ptrContainer->GetAdapter<SignAdapterDetectSSD>(adapter_detect_ssd);
				detections = detectAdapter->Detect(image);
				
				if (detectionslast.size() != 0)
				{
					if ((imgNum - detectionslast[0].frameId) > 5)//标牌识别结果超过5帧，清空缓存结果
						detectionslast.clear();
				}
				if (detections.size() > 0)
				{
					SignAdapterClassify* classifyAdapter = m_ptrContainer->GetAdapter<SignAdapterClassify>(adapter_classify);
					if (detectionslast.size() == 0)
					{
						for (int i = 0; i < detections.size(); ++i)
						{
							ISign::Sign classiy = classifyAdapter->MixClassify(image(detections[i].box));
							detections[i].frameId = imgNum;
							detections[i].smallClassName = classiy.smallClassName;
							detections[i].label = classiy.label;
							if (detections[i].label == -1)//剔除分类为-1的目标
								continue;
							detectionslast.push_back(detections[i]);
						}
						imagelast = curImgName;
					}
					else
					{
						for (int i = 0; i < detections.size(); ++i)
						{
							std::chrono::steady_clock::time_point startClass = std::chrono::steady_clock::now();

							ISign::Sign classiy = classifyAdapter->MixClassify(image(detections[i].box));

							std::chrono::steady_clock::time_point endClass = std::chrono::steady_clock::now();

							std::cout << "One frame classify time: "
								<< std::chrono::duration_cast<std::chrono::milliseconds>(endClass - startClass).count()
								<< "ms!\n";
							detections[i].frameId = imgNum;
							detections[i].smallClassName = classiy.smallClassName;
							detections[i].label = classiy.label;
							if (detections[i].label == -1)//剔除分类为-1的目标
								continue;
							else
							{
								bool isCalRt = false;
								vector<Point2f> lastpixs, curpixs;
								for (int j = 0; j < detectionslast.size(); ++j)
								{
									if (detections[i].label == detectionslast[j].label)
									{
										isCalRt = true;
										Point2f lastpix, curpix;
										lastpix.x = detectionslast[j].box.x * 2 + detectionslast[j].box.width;
										lastpix.y = detectionslast[j].box.y * 2 + detectionslast[j].box.height;
										lastpixs.push_back(lastpix);
										curpix.x = detections[i].box.x * 2 + detections[i].box.width;
										curpix.y = detections[i].box.y * 2 + detections[i].box.height;
										curpixs.push_back(curpix);
									}
								}
								if (isCalRt)
								{
									double curtime, lasttime;
									TPA curpos, lastpos;
								
									if (!getImageGPSTimeFromFileNameImpl(imagelast.c_str(), lasttime))
									{
										std::cout << "get last time failed!" << std::endl;
										return;
									}
									//std::cout << "lasttime时间：" << lasttime << std::endl;
									if (!imagePosT.getTPAByTime(lasttime, &lastpos))
									{
										std::cout << "get last TPA failed!" << std::endl;
										return;
									}
									
									if (!getImageGPSTimeFromFileNameImpl(curImgName.c_str(), curtime))
									{
										std::cout << "get cur time failed!" << std::endl;
										return;
									}
									//std::cout << "curtime时间：" << curtime << std::endl;
									if (!imagePosT.getTPAByTime(curtime, &curpos))
									{
										std::cout << "get cur TPA failed!" << std::endl;
										return;
									}
									Mat Rimu2navt1 = CalcRimu2nav(-lastpos.heading, lastpos.pitch, lastpos.roll);
									Mat Rimu2navt2 = CalcRimu2nav(-curpos.heading, curpos.pitch, curpos.roll);

									BLHCoordinate blht1;
									BLHCoordinate blht2;
									blht1.latitude = lastpos.latitude;
									blht1.longitude = lastpos.longitude;
									blht1.altitude = lastpos.height;

									blht2.latitude = curpos.latitude;
									blht2.longitude = curpos.longitude;
									blht2.altitude = curpos.height;

									Mat XYZ2Navt1 = CalcRxyz2nav(blht1.latitude, blht1.longitude);
									Mat XYZ2Navt2 = CalcRxyz2nav(blht2.latitude, blht2.longitude);
									Mat Rimu2xyzt1 = XYZ2Navt1.t()*Rimu2navt1;
									Mat Rimu2xyzt2 = XYZ2Navt2.t()*Rimu2navt2;
									Mat Rt1t2 = Rcam2imu.t()*Rimu2xyzt2.t()*Rimu2xyzt1*Rcam2imu;

									XYZCoordinate xyzt1;
									XYZCoordinate xyzt2;
									xyzt1 = BLH_to_XYZ(blht1);
									xyzt2 = BLH_to_XYZ(blht2);

									Mat resPInv = (Mat_<double>(3, 1) << xyzt2.x, xyzt2.y, xyzt2.z);
									Mat Timu2xyz = (Mat_<double>(3, 1) << xyzt1.x, xyzt1.y, xyzt1.z);
									Mat Pcamt2 = Rimu2xyzt2*Tcam2imu + resPInv;
									Mat imuPcamInv = Rimu2xyzt1.t()*Pcamt2 - Rimu2xyzt1.t()*Timu2xyz;
									Mat camPcamInv = Rcam2imu.t()*imuPcamInv - Rcam2imu.t()*Tcam2imu;
									Mat R = Rt1t2;
									Mat t = -Rt1t2*camPcamInv;
									Mat T1 = (Mat_<double>(3, 4) << 1, 0, 0, 0,//左相机变换矩阵
										0, 1, 0, 0,
										0, 0, 1, 0);
									Mat T2 = (Mat_<double>(3, 4) <<//右相机变换矩阵
										R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
										R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
										R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

									Mat pos4d;
									std::vector<Point3d> points;//保存特征点三维信息
									cv::triangulatePoints(KLp*T1, KLp*T2, lastpixs, curpixs, pos4d);
									for (int i = 0; i<pos4d.cols; i++)
									{
										Mat x = pos4d.col(i);
										x /= x.at<float>(3, 0); // 归一化
										Point3d p(
											x.at<float>(0, 0),
											x.at<float>(1, 0),
											x.at<float>(2, 0));
										points.push_back(p);
										//cout << p << endl;
									}
									
									for (auto ps : points)
									{
										Mat Pco = (Mat_<double>(3, 1) << ps.x, ps.y, ps.z);

										BLHCoordinate blh_m;
										XYZCoordinate xyz_m;
										Mat Sxyz_m = Rimu2xyzt1*(Rcam2imu*Pco + Tcam2imu) + Timu2xyz;
										xyz_m.x = Sxyz_m.at<double>(0, 0);
										xyz_m.y = Sxyz_m.at<double>(1, 0);
										xyz_m.z = Sxyz_m.at<double>(2, 0);
										blh_m = XYZ_to_BLH(xyz_m);
										//cout << setiosflags(ios::fixed) << setprecision(11) << blh_m.latitude << ',' << blh_m.longitude << ',' << blh_m.altitude << endl;
									}
									
								}	
								
							}
						}
						detectionslast.clear();
						for (int i = 0; i < detections.size(); ++i)
						{
							ISign::Sign classiy = classifyAdapter->MixClassify(image(detections[i].box));
							detections[i].frameId = imgNum;
							detections[i].smallClassName = classiy.smallClassName;
							detections[i].label = classiy.label;
							if (detections[i].label == -1)//剔除分类为-1的目标
								continue;
							detectionslast.push_back(detections[i]);
						}
						imagelast = curImgName;
					}
					
				}
			}
		}
	}



