#include "CynSingleCircleSubpixelEdge.h"
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include<cmath>
#include <QtCore/qmath.h>

#include <QDebug>


using namespace cv;
using namespace std;

CynSingleCircleSubpixelEdge::CynSingleCircleSubpixelEdge(Mat srcImage, int number, double step, int cannyThreshold)
    :_srcImage(srcImage),_number(number),_step(step), _cannyThreshold(cannyThreshold)
{
}

void CynSingleCircleSubpixelEdge::MainEntry()
{
    vector<Point> contourCanny;
    Mat edgeGrad_x, edgeGrad_y;

    PreProcessImage(contourCanny, edgeGrad_x, edgeGrad_y);

    vector<Vec2d> vGradDirection(contourCanny.size());
    CalcGradDirection(vGradDirection, edgeGrad_x, edgeGrad_y, contourCanny);

    SubpixelLocation_np(_srcImage, vGradDirection, contourCanny, _number, _step);//coutourCanny是边缘点的坐标集合，

}

int CynSingleCircleSubpixelEdge::PreProcessImage(vector<Point>& contourCanny, Mat& edgeGrad_x, Mat& edgeGrad_y)
{
    Mat src = _srcImage;

    Mat image;
    GaussianBlur(src, image, Size(3, 3), 0, 0, BORDER_DEFAULT);
    /*
     * 各种滤波方式的比较，包括：
     * 双边和高斯
     * Sobel和Scharr
     */
//    bilateralFilter(src,image,10,10,10);
//    bilateralFilter(src,image,5,5,5);
//        Mat resizeone=Mat::zeros(512,512,CV_8UC3);
//        resize(image,resizeone,resizeone.size());
//        imshow("123",resizeone);
    int ddepth = CV_32F;
    Mat grad_x, grad_y;
    Sobel(image, grad_x, ddepth, 1, 0, 3, 1, 0, BORDER_DEFAULT);
    Sobel(image, grad_y, ddepth, 0, 1, 3, 1, 0, BORDER_DEFAULT);


//    Scharr(image, grad_x, ddepth, 1, 0, 1, 0, BORDER_DEFAULT);
//    Scharr(image, grad_y, ddepth, 0, 1, 1, 0, BORDER_DEFAULT);

    Mat edgeCanny;
    int cannyThreshold = _cannyThreshold;
    Canny(image, edgeCanny, cannyThreshold, cannyThreshold * 2, 3);



    vector<vector<Point> > contoursCanny;
    findContours(edgeCanny, contoursCanny, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    Mat matContoursCanny(image.size().height, image.size().width, image.depth(), Scalar(0));

    contourCanny = contoursCanny[0];
    DrawContourToMat(matContoursCanny, contourCanny, 255);


    grad_x.copyTo(edgeGrad_x, matContoursCanny);

    grad_y.copyTo(edgeGrad_y, matContoursCanny);





    return 1;
}




//亚像素边缘定位，用灰度差分后，再拟合抛物线，一阶导为0处即亚像素点
void CynSingleCircleSubpixelEdge::SubpixelLocation_np(Mat image, vector<Vec2d>& vGradDirection, vector<Point>& contour,int n,double step)
{
    //image是原图，vGradDirection是梯度方向，contour是边缘点坐标，number和step是private参数，n=5，step=1.5
    /*
     * 做亚像素用的是原图，再用滤波后的图做一下进行比较
     */

	ofstream outfile("edgePoints_np_.txt", ios::out);

	double* pLamda       = new double[n];
	double* pNewX        = new double[n];
	double* pNewY        = new double[n];
	double* pGrayVal	 = new double[n];
	double* pLamda2      = new double[n-1];
	double* pDiffGrayVal = new double[n-1];

    int hn = (n-1) / 2;      //hn=2
	for (int i = 0; i < n;i++){
		pLamda[i] = (i - hn) * step;
//        qDebug()<<pLamda[i];

	}

	for (int i = 0; i < n-1; i++){
		pLamda2[i] = (pLamda[i] + pLamda[i+1])*0.5;
//        qDebug()<<pLamda2[i];
	}

	for (unsigned int k = 0; k < contour.size(); k++)
	{
        if(k==1210)
        {
            int adsadasd=1;
        }
		int y = contour[k].y;
		int x = contour[k].x;
		double g0 = image.at<uchar>(y, x);

        double nx = vGradDirection[k].val[0];
//        double cosx=qAcos(nx);
//        cosx=cosx/3.1415*180;
//        qDebug()<<cosx;
		double ny = vGradDirection[k].val[1];

		for (int i = 0; i < n; i++)
		{
			if (i == hn){ //整像素点自身直接取值
				pNewX[i] = x;
				pNewY[i] = y;
				pGrayVal[i] = double(image.at<uchar>(y, x));
			}
			else{
				pNewX[i] = x + pLamda[i] * nx;
				pNewY[i] = y + pLamda[i] * ny;
                /*
                 * 注意pNew是否位置超过Mat范围，干脆拼接时不取四周，取中间
                 */
				BiLinearInterpolation(image, pNewX[i], pNewY[i], pGrayVal[i]);
			}
		}

		for (int i = 0; i < n - 1; i++)	{
			pDiffGrayVal[i] = pGrayVal[i + 1] - pGrayVal[i];
		}

		double a, b, c;
        fittingParabola(pLamda2, pDiffGrayVal, n - 1, a, b, c);

        if(a==0) continue;
		double lamda_i = -0.5*b / a; //导数为0点，即为亚像素边缘



		double xSubPixel = x + lamda_i * nx;
		double ySubPixel = y + lamda_i * ny;

		Point2f edge_point_f(xSubPixel, ySubPixel);
		edgePoints_image_f.push_back(edge_point_f);

		sub_edgePoint1 edge_point1(xSubPixel, ySubPixel);
		edgePoints_image.push_back(edge_point1);
		
		outfile << x << "\t" << y << "\t" << xSubPixel << "\t" << ySubPixel << endl;
	}

	delete[] pLamda;
	delete[] pNewX;
	delete[] pNewY;
	delete[] pGrayVal;
	delete[] pDiffGrayVal;
}

//a*x^2+b*x+c=y
void CynSingleCircleSubpixelEdge::fittingParabola(double x[], double y[], int n, double& a, double& b, double& c)
{
    /*x[]是5个等分点相邻两个点的中间点，y[]是5个等分点相邻两个点的灰度值的差
     *
     */
	//A*x=B,最小二乘解法：x= inv(A'A)*A'b
	Mat A, At, AtA, invAtA, B, Xtemp,X;
	A.create(n,3,CV_64FC1);
	B.create(n,1, CV_64FC1);
	for (int i = 0; i < n; i++)
	{
        A.at<double>(i,0) = x[i]*x[i];
//        qDebug()<<x[i];
        A.at<double>(i,1) = x[i];
        A.at<double>(i,2) = 1.0;
		
        B.at<double>(i, 0) = y[i];
//        qDebug()<<y[i];
	}

	At = A.t();
	AtA = At*A;
	invAtA = AtA.inv();

	Xtemp = At * B;
	X = invAtA * Xtemp;

    a = X.at<double>(0, 0);
    b = X.at<double>(1, 0);
    c = X.at<double>(2, 0);
}


void CynSingleCircleSubpixelEdge::DrawContourToMat(Mat image, vector<Point>& contour, Scalar value)
{
	//	image.empty();
	for (unsigned int k = 0; k < contour.size(); k++)
	{
		int r = contour[k].y;
		int c = contour[k].x;
		image.at<uchar>(r, c) = static_cast<uchar>(value.val[0]);
	}
}

void CynSingleCircleSubpixelEdge::CalcGradDirection(vector<Vec2d>& vGradDirection, Mat Gx, Mat Gy, vector<Point>& contour)
{

	for (unsigned int k = 0; k < contour.size(); k++)
	{
		int r = contour[k].y;
		int c = contour[k].x;
		float gxi = Gx.at<float>(r, c);
        float gyi = Gy.at<float>(r, c);

		double amplitude = sqrt(gxi*gxi + gyi*gyi);
		double nx;
		double ny;
		if (amplitude==0)
		{
			nx = 0;
			ny = 0;
//            qDebug()<<"123";
		} 
		else
		{
			nx = double(gxi) / amplitude;
			ny = double(gyi) / amplitude;
//            qDebug()<<nx;
		}
		

		vGradDirection[k] = Vec2d(nx, ny);
	}
}



//双线性灰度插值
void CynSingleCircleSubpixelEdge::BiLinearInterpolation(Mat image, double xNew, double yNew, double& grayValue)
//=====================================================================================
{
//    if (xNew < 0.0 || yNew < 0.0||xNew>image.rows||yNew>image.cols) { return; }
    int x0, y0, x1, y1;
    x0 = (int)xNew;
    x1 = x0 + 1; // i+1
    y0 = (int)yNew;
    y1 = y0 + 1; // j+1

    double u, v;
    u = xNew - (double)x0; // distance from pixel(i0,j0)
    v = yNew - (double)y0;

    double g00, g10, g01, g11;

    if(x0<0) x0=0;
    if(x0>=image.cols) x0=image.cols-1;
    if(x1<0) x1=0;
    if(x1>=image.cols) x1=image.cols-1;
    if(y0<0) y0=0;
    if(y0>=image.rows) y0=image.rows-1;
    if(y1<0) y1=0;
    if(y1>=image.rows) y1=image.rows-1;
    g00 = image.at<uchar>(y0, x0);
    g10 = image.at<uchar>(y0, x1);
    g01 = image.at<uchar>(y1, x0);
    g11 = image.at<uchar>(y1, x0);

    grayValue = (1 - u)*(1 - v)*g00 + u*(1 - v)*g10 + (1 - u)*v*g01 + u*v*g11;
}

