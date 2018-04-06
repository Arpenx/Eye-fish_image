/*鱼眼校正之四分和圆柱展开，导yml*/
/*自行改参数：rr，RR，ok，oc*/
#include <opencv2/opencv.hpp>
#include "iostream"
#include "xp.h"
using namespace std;
using namespace cv;
float f;

int main()
{
    /*数据区*/
    float rr = 68.0f;//135;//225.0f;
    float RR = 300.0f; //616;//974.0f;
    float ok = 300.0f; //535;//779.0f;//圆心y坐标
    float oc = 300.0f;// 616;//974.0f;//圆心x坐标
    float thetamax = 89.0f * CV_PI / 180.0f;
    f = RR / thetamax;
    float thetamin = rr / f;
    string imgname = "/Users/arpenx/Documents/project/qtcreator/Eyefish/Eye-fish_image/camera.bmp";

    /*读图*/
    Mat img = imread(imgname);
    cv::namedWindow("src", CV_GUI_EXPANDED);
    cv::imshow("src", img);
    int row;
    int col;
    //cout << img.cols << '\t' << img.rows << endl;
    ////////////////////////////////////////////////////////////////////////////////////////////
    /*圆柱*/
    row = RR / tan(thetamin);
    col = RR * 2 * CV_PI;
    cv::Mat mapx2(row, col, CV_32FC1, cv::Scalar(0));
    cv::Mat mapy2(row, col, CV_32FC1, cv::Scalar(0));
    cal_mapping_two(row, col, mapx2, mapy2, RR);

    //remap and then save the image
    mapx2 = mapx2 + oc;
    mapy2 = mapy2 + ok;
    cv::Mat nimg2;
    cv::remap(img, nimg2, mapx2, mapy2, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::namedWindow("twoxp", CV_GUI_EXPANDED);
    cv::flip(nimg2,nimg2, 0);
    cv::imshow("twoxp", nimg2);
    cv::imwrite("/Users/arpenx/Documents/project/qtcreator/Eyefish/Eye-fish_image/twoxp.bmp", nimg2);
    ///////////////////////////////////////////////////////////////////////////////////////////////
    /*四分*/
    /*计算3个点校正后的数据，返回能存下他的最小矩形的长宽*/
    float pos1[] = { thetamax, thetamax, thetamin };
    float pos2[] = { CV_PI/4, CV_PI / 2, CV_PI / 2 };
    float paras[] = { 0, 0 };
    cal_subview(pos1, pos2, paras);
    /*计算映射*/
    row = paras[1]*2;
    col = paras[0]*2;
    cv::Mat mapx1(row, col, CV_32FC1, cv::Scalar(0));
    cv::Mat mapy1(row, col, CV_32FC1, cv::Scalar(0));
    cal_mapping_four(row, col, paras, mapx1, mapy1);
    mapx1 +=  oc;
    mapy1 +=  ok;

    //remap and then save the image
    cv::Mat nimg1;
    cv::remap(img, nimg1, mapx1, mapy1, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::namedWindow("fourxp", CV_GUI_EXPANDED);
    cv::imshow("fourxp", nimg1);
    cv::imwrite("/Users/arpenx/Documents/project/qtcreator/Eyefish/Eye-fish_image/fourxp.bmp", nimg1);

    //////////////////////////////////////////////////////////////////////////////////////
    /*导xml*/
    //FileStorage fs;
    //fs.open("/Users/arpenx/Documents/project/eyeFish_images/demo.xml", FileStorage::WRITE);
    //fs << "mapx1" << mapx1 << "mapy1" << mapy1 << "mapx2" << mapx2 << "mapy2" << mapy2;
    //fs.release();

    waitKey();
    return 0;

}
