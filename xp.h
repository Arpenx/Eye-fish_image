#ifndef XP_H
#define XP_H

/*鱼眼校正之四分和圆柱展开，导yml*/
#pragma once
#include "opencv2/opencv.hpp"
#include <time.h>
void sph2cart(const cv::Mat &phi/*2D上与x轴的夹角*/, const cv::Mat &theta/*球面上点与球心连线与z轴的夹角*/, cv::Mat &x, cv::Mat &y, cv::Mat &z);/*由3D的两夹角转3D空间直角坐标*/
void rotate3D(const float &alpha, const float &beta, const float &gamma, cv::Mat &Rx, cv::Mat &Ry, cv::Mat &Rz);/*3D旋转矩阵*/
void cal_subview(float(&pos1)[3], float(&pos2)[3], float(&paras)[2]);
void cart2sph(const cv::Mat &x, const cv::Mat &y, const cv::Mat &z, cv::Mat &phi, cv::Mat &theta);
void cal_mapping_four(int &row, int &col, float(&paras)[2], cv::Mat &mapx1, cv::Mat &mapy1);
void cal_mapping_two(int &row, int &col, cv::Mat &mapx1, cv::Mat &mapy1, int R);
void meshgrid(const cv::Range &xgv, const cv::Range &ygv, cv::Mat &X, cv::Mat &Y);/*把Range映射为Mat*/
extern float  f;

#endif // XP_H
