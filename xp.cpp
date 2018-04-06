/*鱼眼校正之四分和圆柱展开，导yml*/
#include "xp.h"

void sph2cart(const cv::Mat &phi/*2D上与x轴的夹角*/, const cv::Mat &theta/*球面上点与球心连线与z轴的夹角*/, cv::Mat &x, cv::Mat &y, cv::Mat &z)/*由3D的两夹角转3D空间直角坐标*/
{
    for (int j = 0; j < phi.cols; j++)
    for (int i = 0; i < phi.rows; i++)
    {
        if (!(phi.at<float>(i, j) == 0 && theta.at<float>(i, j) == 0))
        {
            x.at<float>(i, j) = std::sin(theta.at<float>(i, j)) * std::cos(phi.at<float>(i, j));
            y.at<float>(i, j) = std::sin(theta.at<float>(i, j)) * std::sin(phi.at<float>(i, j));
            z.at<float>(i, j) = std::cos(theta.at<float>(i, j));
        }
    }
}

void rotate3D(const float &alpha, const float &beta, const float &gamma, cv::Mat &Rx, cv::Mat &Ry, cv::Mat &Rz)/*3D旋转矩阵*/
{
    Rx.at<float>(0, 0) = 1; Rx.at<float>(0, 1) = 0;			  Rx.at<float>(0, 2) = 0;
    Rx.at<float>(1, 0) = 0; Rx.at<float>(1, 1) = cos(alpha);  Rx.at<float>(1, 2) = sin(alpha);
    Rx.at<float>(2, 0) = 0; Rx.at<float>(2, 1) = -sin(alpha); Rx.at<float>(2, 2) = cos(alpha);

    Ry.at<float>(0, 0) = cos(beta); Ry.at<float>(0, 1) = 0;	Ry.at<float>(0, 2) = -sin(beta);
    Ry.at<float>(1, 0) = 0;			Ry.at<float>(1, 1) = 1; Ry.at<float>(1, 2) = 0;
    Ry.at<float>(2, 0) = sin(beta); Ry.at<float>(2, 1) = 0; Ry.at<float>(2, 2) = cos(beta);

    Rz.at<float>(0, 0) = cos(gamma);  Rz.at<float>(0, 1) = sin(gamma); Rz.at<float>(0, 2) = 0;
    Rz.at<float>(1, 0) = -sin(gamma); Rz.at<float>(1, 1) = cos(gamma); Rz.at<float>(1, 2) = 0;
    Rz.at<float>(2, 0) = 0;			  Rz.at<float>(2, 1) = 0;		   Rz.at<float>(2, 2) = 1;
}

void cart2sph(const cv::Mat &x, const cv::Mat &y, const cv::Mat &z, cv::Mat &phi, cv::Mat &theta)/*由3D空间直角坐标转3D的两夹角*/
{
    for (int j = 0; j < x.cols; j++)
    for (int i = 0; i < x.rows; i++)
    {
        float r =  sqrt(x.at<float>(i, j)*x.at<float>(i, j) + y.at<float>(i, j)*y.at<float>(i, j) + z.at<float>(i, j)*z.at<float>(i, j));
        if (r != 0)
        {
            phi.at<float>(i, j) = atan2(y.at<float>(i, j), x.at<float>(i, j));/*2D上与x轴的夹角*/
            theta.at<float>(i, j) = acos(z.at<float>(i, j) / r);/*球面上点与球心连线与z轴的夹角*/
        }
    }
}

void cal_subview(float(&pos1)[3], float(&pos2)[3], float(&paras)[2])//得到四分之一鱼眼图矫正后的长宽
{
    /*2D到3D*/
    cv::Mat pos_theta(3, 1, CV_32FC1, pos1);
    cv::Mat pos_phi(3, 1, CV_32FC1, pos2);
    cv::Mat pos_x(3, 1, CV_32FC1, cv::Scalar(0));
    cv::Mat pos_y(3, 1, CV_32FC1, cv::Scalar(0));
    cv::Mat pos_z(3, 1, CV_32FC1, cv::Scalar(0));
    sph2cart(pos_phi, pos_theta, pos_x, pos_y, pos_z);


    /*获取旋转矩阵*/
    cv::Mat Rx(3, 3, CV_32FC1);
    cv::Mat Ry(3, 3, CV_32FC1);
    cv::Mat Rz(3, 3, CV_32FC1);
    rotate3D(CV_PI / 4,0, CV_PI / 4, Rx, Ry, Rz);


    /*获取球面上点的两个角度*/
    cv::Mat stdxyz(pos_x.rows, 3, pos_x.type());
    pos_x.copyTo(stdxyz.col(0));
    pos_y.copyTo(stdxyz.col(1));
    pos_z.copyTo(stdxyz.col(2));
    stdxyz = stdxyz * Rz* Rx;
    stdxyz.col(0).copyTo(pos_x);
    stdxyz.col(1).copyTo(pos_y);
    stdxyz.col(2).copyTo(pos_z);
    cart2sph(pos_x, pos_y, pos_z, pos_phi, pos_theta);


    cv::Mat pos_r(pos_x.rows, pos_x.cols, CV_32FC1);

    /*成像模型（r=f*tan（theta））求R*/
    for (int i = 0; i < pos_r.rows; i++)
        pos_r.at<float>(i, 0) = f * tan(pos_theta.at<float>(i, 0));

    /*2D极坐标转直角坐标*/
    polarToCart(pos_r, pos_phi, pos_x, pos_y);
    paras[0] = 2.0f * abs(pos_x.at<float>(1, 0));
    paras[1] = abs(pos_y.at<float>(0, 0) - pos_y.at<float>(2, 0));
}

void meshgrid(const cv::Range &xgv, const cv::Range &ygv, cv::Mat &X, cv::Mat &Y)/*把Range映射为Mat*/
{
    std::vector<float> t_x, t_y;
    for (int i = xgv.start; i <= xgv.end; i++) t_x.push_back(float(i));
    for (int j = ygv.start; j <= ygv.end; j++) t_y.push_back(float(j));
    cv::repeat(cv::Mat(t_x).t(), t_y.size(), 1, X);/*repeat 函数是将t_x扩展为与t_y相同大小的矩阵，相当于Matlab中的repmat*/
    cv::repeat(cv::Mat(t_y), 1, t_x.size(), Y);
}

void cal_mapping_four(int &row, int &col, float(&paras)[2], cv::Mat &mapx1, cv::Mat &mapy1)
{
    float width = paras[0];
    float height = paras[1];
    cv::Mat nX, nY;
    //旋转角度
    std::vector<float> p1,p2;
    p1.push_back(CV_PI / 4);
    p1.push_back(CV_PI / 4);
    p1.push_back(-CV_PI / 4);
    p1.push_back(-CV_PI / 4);

    p2.push_back(-CV_PI / 4);
    p2.push_back(CV_PI / 4);
    p2.push_back(CV_PI / 4);
    p2.push_back(-CV_PI / 4);

    for (int k = 0; k < 4; k++)
    {
        /*中心化*/
        cv::Mat X, Y;
        meshgrid(cv::Range(1, col), cv::Range(1, row), X, Y);
        /*直角坐标中心化*/
        cv::Mat c_X = X - col/2;//右-》
        cv::Mat c_Y = Y - row/2;//下

        float centerx, centery;

        switch (k + 1)
        {
        case 1:
            centerx = - width/ 2;
            centery = - height/ 2;
            break;
        case 2:
            centerx =  width/ 2;
            centery = - height/ 2;
            break;
        case 3:
            centerx = - width/ 2;
            centery = height / 2;
            break;
        case 4:
            centerx =  width/ 2;
            centery =  height/ 2;
            break;
        }
        /*四分之一图 中心化*/
        c_X = c_X - centerx;
        c_Y = c_Y - centery;
        //float x1 = centerx - width / 2;
        //float y1= centery - height / 2;
        //float x2 = centerx + width / 2;
        //float y2 = centery + height / 2;
        //cv::Mat dst_X, dst_Y;
        //c_X.copyTo(dst_X(cv::Rect(x1, y1, x2, y2)));
        //c_Y.copyTo(dst_Y(cv::Rect(x1, y1, x2, y2)));

        cv::Mat temp(row, col, CV_32FC1, cv::Scalar(0));

        ///*临时矩阵 存储有效位置*/
        for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
        {
            if (c_X.at<float>(i, j)<width / 2 && c_X.at<float>(i, j)>-width / 2 && c_Y.at<float>(i, j)<height/2
                && c_Y.at<float>(i, j)>-height/2)
            {
                temp.at<float>(i, j) = 1;
            }
        }

        /*直角坐标转极坐标*/
        cv::Mat phi, rou;
        cv::cartToPolar(c_X, c_Y, rou, phi);
    //	rou = rou * 2;

        /*球坐标xyz*/
        cv::Mat theta(row, col, CV_32FC1);
        for (int j = 0; j < col; j++)
        for (int i = 0; i < row; i++)
            theta.at<float>(i, j) = atan2(rou.at<float>(i, j), f);

        cv::Mat XX(row, col, CV_32FC1, cv::Scalar(0));
        cv::Mat YY(row, col, CV_32FC1, cv::Scalar(0));
        cv::Mat ZZ(row, col, CV_32FC1, cv::Scalar(0));
        sph2cart(phi, theta, XX, YY, ZZ);
        /*求旋转矩阵*/
        cv::Mat Rx(3, 3, CV_32FC1);
        cv::Mat Ry(3, 3, CV_32FC1);
        cv::Mat Rz(3, 3, CV_32FC1);
        rotate3D(p1.at(k), 0, p2.at(k), Rx, Ry, Rz);

        XX = XX.mul(temp); YY = YY.mul(temp); ZZ = ZZ.mul(temp);//将超出范围的值赋值为0，范围内不变

        /*旋转*/
        cv::Mat XYZ(row*col, 3, XX.type());
        cv::Mat TXX = XX.reshape(0, row*col);
        cv::Mat TYY = YY.reshape(0, row*col);
        cv::Mat TZZ = ZZ.reshape(0, row*col);
        TXX.copyTo(XYZ.col(0));
        TYY.copyTo(XYZ.col(1));
        TZZ.copyTo(XYZ.col(2));
        XYZ = XYZ *Rx* Rz;
        XYZ.col(0).copyTo(TXX);
        XYZ.col(1).copyTo(TYY);
        XYZ.col(2).copyTo(TZZ);
        cv::Mat nXX(XX.size(), XX.type());
        cv::Mat nYY(XX.size(), XX.type());
        cv::Mat nZZ(XX.size(), XX.type());
        TXX.reshape(0, row).copyTo(nXX);
        TYY.reshape(0, row).copyTo(nYY);
        TZZ.reshape(0, row).copyTo(nZZ);

        cv::Mat nphi(row, col, CV_32FC1, cv::Scalar(0));
        cv::Mat ntheta(row, col, CV_32FC1, cv::Scalar(0));
        cart2sph(XX, YY, ZZ, nphi, ntheta);

        cv::Mat nrou(row, col, CV_32FC1);
        for (int j = 0; j < col; j++)
        for (int i = 0; i < row; i++)
            nrou.at<float>(i, j) =	f * ntheta.at<float>(i, j);
        polarToCart(nrou, nphi, nX, nY);
        /*将每次成像坐标对应存在map1中*/
        mapx1 = mapx1+nX;
        mapy1 = mapy1+nY;
    }
}

void cal_mapping_two(int &row, int &col, cv::Mat &mapx1, cv::Mat &mapy1, int R)
{
    cv::Mat X, Y;
    meshgrid(cv::Range(1, col), cv::Range(1, row), X, Y);

    cv::Mat phi = X / R;
    cv::Mat theta(row, col, CV_32FC1);
    for (int j = 0; j < col; j++)
    for (int i = 0; i < row; i++)
        theta.at<float>(i, j) = atan(R / Y.at<float>(i, j));

    cv::Mat rou = f * theta;
    cv::Mat nX, nY;
    cv::polarToCart(rou, phi, nX, nY);
    mapx1 = nX;
    mapy1 = nY;

}
