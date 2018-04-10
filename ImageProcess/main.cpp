#include "mainwindow.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include "CynSingleCircleSubpixelEdge.h"
#include "matchimage.h"
#include <QDebug>
#include <QtCore/qmath.h>

using namespace std;
using namespace cv;

#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma once
#include <iostream>
#include<cmath>

#include <QDesktopWidget>
#include "qt_windows.h"

LARGE_INTEGER g_fre;
LARGE_INTEGER g_start_time;
LARGE_INTEGER g_end_time;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    QueryPerformanceFrequency(&g_fre);
    QueryPerformanceCounter(&g_start_time);

//    Mat image1 = imread("matching1.bmp", 0);
   // Mat image = imread("222.png", 0);
    Mat match1 = imread("4-9-1.bmp", 0);
    Mat match2 = imread("4-9-2.bmp",0);
    Mat match3 = imread("4-9-3.bmp",0);
    Mat match4 = imread("4-9-4.bmp",0);
    Mat match5 = imread("4-9-5.bmp",0);
//    Mat match6 = imread("4-9-6.bmp",0);

//    CynSingleCircleSubpixelEdge ynED(src, 5, 1.5, 140);
//    ynED.MainEntry();

    QueryPerformanceCounter(&g_end_time);
    double milliseconds=((double)g_end_time.QuadPart-(double)g_start_time.QuadPart)/
            ((double)g_fre.QuadPart);

   //qDebug()<<milliseconds;

//    ofstream Fs("test.txt");
//    if (!Fs.is_open())
//    {
//        std::cout << "open file error!" << std::endl;
//        return 0;
//    }

//    //循环输出
//    int height = M.rows;
//    int width = M.cols;
//    for (int i = 0; i<height; i++)
//    {
//        for (int j = 0; j<width; j++)
//        {
//            Fs << (int)M.ptr<uchar>(i)[j] << '\t';
//            //不加类型转换用txt打开是字符
//        }
//        Fs << std::endl;
//    }
//    Fs.close();


    MatchImage testone(match1);
//    testone.AddImage(match2,1000);
    testone.AddImage(match2,1000);
    testone.AddImage(match3,1000);
//    testone.AddImage(match4,1000);


    Mat amen;
    amen = testone.GetMatchingImage();
    imwrite("matching.bmp",amen);

//    Mat resizeone=Mat::zeros(1000,1200,CV_8UC1);
//    resize(match2,resizeone,resizeone.size());
//    imshow("111",resizeone);
//    Mat resizeone3=Mat::zeros(1000,1200,CV_8UC1);
//    resize(match1,resizeone3,resizeone3.size());
//    imshow("222",resizeone3);
    Mat resizeone2=Mat::zeros(1000,1200,CV_8UC1);
    resize(amen,resizeone2,resizeone2.size());
    imshow("333",resizeone2);












    //w.show();
    return a.exec();
}


