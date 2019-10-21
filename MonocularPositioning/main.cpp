#include <fstream>
#include <algorithm>
#include "Monocular_Utils.h"
#include "Monocular_Positioning.h"
#include "Monocular_CoorTransform.h"
#include "Monocular_Config.h"

using namespace std;
using namespace cv;


#define OUTPUTPATH					"/Users/TLG/Documents/GitHub/MonocularPositioning/result"			//"E:/Monocular/Positioining/singleresult"							//输出目录



Point3d GetCenterValue(const Point3d &pre, const Point3d &cur)
{
	return ( pre + cur ) / 2.0;
}

struct PtItem
{
    Point2f       pixel;
    BLHCoordinate gps;
};


extern double GetDistance(BLHCoordinate pre, BLHCoordinate cur);
//{
//    Point3d xyz1 = M_CoorTrans::BLH_to_XYZ(pre);
//    Point3d xyz2 = M_CoorTrans::BLH_to_XYZ(cur);
//    Point3d rst{ xyz2.x - xyz1.x, xyz2.y - xyz1.y, xyz2.z - xyz1.z };
//    return sqrt(rst.x * rst.x + rst.y * rst.y + rst.z * rst.z);
//}

inline Point3d PtFromMat(const Mat &m)
{
    return Point3d(m.at<double>(0,0),
                   m.at<double>(1,0),
                   m.at<double>(2,0));
}

inline Mat MatFromPt(const Point3d &pt)
{
    return (Mat_<double>(3,1) <<pt.x,pt.y,pt.z );
}

int main()
{
    Monocular_Positioning position(OUTPUTPATH);

    position.Test();

    return 0;
    const BLHCoordinate  framegps =
    {30.446218994375481515,114.47152826339303999,21.467318182944886473};
    PtItem pt1 = {{2445, 308},{30.446226851635728394,114.47132140383631338,26.522761307656764984}};
    PtItem pt2 = {{2698, 301},{30.446245936213536254,114.47131943760462036,26.582286909222602844}};
    PtItem pt3 = {{2570, 303},{30.4462362222766032,114.47132103254763535,26.54782392643392086}};

    PtItem pt4 = {{1793, 487},
        {30.44614554247711169,114.47112313992190025,28.603751753456890583}};
    
    PtItem pt5 = {{2575, 1321},
        {30.446221534534352315,114.47144323750148942,20.035653182305395603}};
    
    PtItem pt6 = {{2659, 69},{30.446248664649665727,114.47128117322539254,29.75151172187179327}};
    
    Point3d xyz1 = M_CoorTrans::BLH_to_XYZ(pt1.gps);
    Point3d xyz2 = M_CoorTrans::BLH_to_XYZ(pt2.gps);
    Point3d xyz3 = M_CoorTrans::BLH_to_XYZ(pt3.gps);
    Point3d xyz4 = M_CoorTrans::BLH_to_XYZ(pt4.gps);
    Point3d xyz5 = M_CoorTrans::BLH_to_XYZ(pt5.gps);
    Point3d xyz6 = M_CoorTrans::BLH_to_XYZ(pt6.gps);

    Mat K = (cv::Mat_<double>(3,3) <<
             2.3122553820951944e+03,0.,2.0424966354370117e+03,
             0.,2.3122553820951944e+03,1.0833294486999512e+03,
             0.,0. ,1.);

    
    Mat Cam2ImuR = (cv::Mat_<double>(3,3) <<  9.9999144974170961e-01, -7.4695110452078660e-04,
                    -4.0672481509404631e-03, 4.0676940220370538e-03,
                    5.9637546372425855e-04, 9.9999154906511556e-01,
                    -7.4451918508354172e-04, -9.9999954320007256e-01,
                    5.9940873310261882e-04);
    Mat Cam2ImuT = (cv::Mat_<double>(3,1) <<  8.9965656397814911e+01, -4.2657811012740410e-02,
                    -2.3306240712445381e-01);
    
    Mat mxyz1 = Cam2ImuR.inv() * ( MatFromPt(xyz1) -  Cam2ImuT );
    Mat mxyz2 = Cam2ImuR.inv() * ( MatFromPt(xyz2) -  Cam2ImuT );
    Mat mxyz3 = Cam2ImuR.inv() * ( MatFromPt(xyz3) -  Cam2ImuT );
    Mat mxyz4 = Cam2ImuR.inv() * ( MatFromPt(xyz4) -  Cam2ImuT );
    Mat mxyz5 = Cam2ImuR.inv() * ( MatFromPt(xyz5) -  Cam2ImuT );
    Mat mxyz6 = Cam2ImuR.inv() * ( MatFromPt(xyz6) -  Cam2ImuT );
    
    xyz1 = PtFromMat(mxyz1);
    xyz2 = PtFromMat(mxyz2);
    xyz3 = PtFromMat(mxyz3);
    xyz4 = PtFromMat(mxyz4);
    xyz5 = PtFromMat(mxyz5);
    xyz6 = PtFromMat(mxyz6);
    
    Point3d xyzcenter = (xyz1 + xyz2 + xyz3 + xyz4 + xyz5 + xyz6) / 6;
    
    xyz1 = xyz1 - xyzcenter;
    xyz2 = xyz2 - xyzcenter;
    xyz3 = xyz3 - xyzcenter;
    xyz4 = xyz4 - xyzcenter;
    xyz5 = xyz5 - xyzcenter;
    xyz6 = xyz6 - xyzcenter;
    
    vector<Point3f> xyzs;
    vector<Point2f> pts;

    xyzs.emplace_back(Point3f(xyz1.x,xyz1.y,xyz1.z));
    xyzs.emplace_back(Point3f(xyz2.x,xyz2.y,xyz2.z));
    xyzs.emplace_back(Point3f(xyz3.x,xyz3.y,xyz3.z));
    xyzs.emplace_back(Point3f(xyz4.x,xyz4.y,xyz4.z));
    xyzs.emplace_back(Point3f(xyz5.x,xyz5.y,xyz5.z));
    xyzs.emplace_back(Point3f(xyz6.x,xyz6.y,xyz6.z));

    pts.emplace_back(pt1.pixel);
    pts.emplace_back(pt2.pixel);
    pts.emplace_back(pt3.pixel);
    pts.emplace_back(pt4.pixel);
    pts.emplace_back(pt5.pixel);
    pts.emplace_back(pt6.pixel);

    cv::Mat R;
    cv::Mat t;
    Mat disff = (Mat_<double>(4,1) <<
                 -7.9480513662361729e-01,
                 -3.4967332387054029e-01,
                 -1.4373246345148066e-04,
                 -1.9744097723913855e-04);
    solvePnP(xyzs, pts, K, cv::noArray(), R, t,false,SOLVEPNP_EPNP);
    cout << R << endl;
    cout << t << endl;
   
    Mat revR;
    Rodrigues(R, revR);
    cout << revR << endl;
    
    Mat rst = -revR.inv() * t;
    
    rst =  Cam2ImuR * (rst + MatFromPt(xyzcenter))+ Cam2ImuT;
    
    Point3d prst = PtFromMat(rst);
    
    BLHCoordinate rstgps = M_CoorTrans::XYZ_to_BLH(prst);
    BLHCoordinate rgps = {30.446218994375481515,114.47152826339303999,21.467318182944886473};
    cout.precision(10);
    cout << "C : " << rstgps.latitude << " " << rstgps.longitude << " " << rstgps.altitude << endl;
    cout << "R : " << rgps.latitude << " " << rgps.longitude << " " << rgps.altitude << endl;
    
    Point3d gaussrst = M_CoorTrans::BLH_to_GaussPrj(rstgps);
    Point3d gaussr   = M_CoorTrans::BLH_to_GaussPrj(rgps);
    
    cout << "distance : " << gaussrst.x - gaussr.x << " " <<
                             gaussrst.y - gaussr.y << endl;
	return 0;
}






