#ifndef CYNSINGLECIRCLESUBPIXELEDGE_H
#define CYNSINGLECIRCLESUBPIXELEDGE_H

#pragma once

#pragma warning(disable: 4819)

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <math.h>
#include <fstream>

using namespace std;
using namespace cv;

struct sub_edgePoint1
{
    double SubPixel_x;
    double SubPixel_y;
    sub_edgePoint1(double x, double y) { SubPixel_x=x; SubPixel_y=y; }
};

class CynSingleCircleSubpixelEdge
{
public:
    CynSingleCircleSubpixelEdge(Mat srcImage,int number,double step, int cannyThreshold=70);
public:
    void MainEntry();
    int PreProcessImage(vector<Point>& contourCanny, Mat& edgeGrad_x, Mat& edgeGrad_y);
    int PreProcessImage();
    int ImageGradient();
    void SubpixelLocation_4p(Mat image, vector<Vec2d>& vGradDirection, vector<Point>& contour);
    void SubpixelLocation_5p(Mat image, vector<Vec2d>& vGradDirection, vector<Point>& contour);
    void SubpixelLocation_np(Mat image, vector<Vec2d>& vGradDirection, vector<Point>& contour, int n=7, double step=1.5);
    vector<sub_edgePoint1> edgePoints_image;
    vector<Point2f> edgePoints_image_f;
protected:
    void BiLinearInterpolation(Mat image, double xNew, double yNew, double& GrayValue);
    void DrawContourToMat(Mat image, vector<Point>& contour, Scalar value = 255);
    void CalcGradDirection(vector<Vec2d>& vGradDirection, Mat Gx, Mat Gy, vector<Point>& contour);
    void DrawGradDirection(Mat image, vector<Vec2d>& vGradDirection, vector<Point>& contour, Scalar color);
    void fittingParabola(double x[], double y[], int n, double& a, double& b, double& c);
    void FastFittingParabola5(double x[], double y[], double& a, double& b, double& c);

private:
    Mat _srcImage; // 源图，8位灰度
    int _cannyThreshold;
    int _number;
    double _step;
};



#endif // CYNSINGLECIRCLESUBPIXELEDGE_H
