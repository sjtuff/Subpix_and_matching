#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "matchimage.h"
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma once
#include <opencv2/opencv.hpp>
#include <QDebug>
using namespace std;
using namespace cv;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QImage cvMat2QImage(const cv::Mat& mat);
    Mat adding_image = imread("123.bmp", 0);






//    ofstream Fs("test.txt");
//    if (!Fs.is_open())
//    {
//        std::cout << "open file error!" << std::endl;
//    }

//    //循环输出
//    int height = edgeCanny1.rows;
//    int width = edgeCanny1.cols;
//    for (int i = 0; i<height; i++)
//    {
//        for (int j = 0; j<width; j++)
//        {
//            if(edgeCanny1.at<uchar>(i, j)!=0)
//            {
//                qDebug()<<edgeCanny1.at<uchar>(i, j);
//            }

//        }
//        Fs << std::endl;
//    }

//    Fs.close();


}

MainWindow::~MainWindow()
{
    delete ui;
}

QImage cvMat2QImage(const cv::Mat& mat)
{
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if(mat.type() == CV_8UC1)
    {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for(int row = 0; row < mat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
    // 8-bits unsigned, NO. OF CHANNELS = 3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    else if(mat.type() == CV_8UC4)
    {
        qDebug() << "CV_8UC4";
        // Copy input Mat
        const uchar *pSrc = (const uchar*)mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

