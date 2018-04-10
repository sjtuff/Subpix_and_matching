#ifndef MATCHIMAGE_H
#define MATCHIMAGE_H

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
class MatchImage
{

public:
    MatchImage(Mat srcImage);
    void AddImage(Mat &adding_image, int image_distance);
    Mat GetMatchingImage();







private:
    Mat _srcImage; // 源图，8位灰度
    int _rows;//初始图的矩阵行数
    Mat _image_match;//拼接后的图
    int _count_image;//拼接的图像数量
    int _row_margine;

};

#endif // MATCHIMAGE_H
