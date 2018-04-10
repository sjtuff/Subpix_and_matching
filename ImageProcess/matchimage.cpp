#include "matchimage.h"
#include "CynSingleCircleSubpixelEdge.h"
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include<cmath>
#include <QImage>
#include <QtCore/qmath.h>
#include <QDebug>
#include<algorithm>

using namespace cv;
using namespace std;


MatchImage::MatchImage(Mat srcImage)
    :_image_match(srcImage)
{
    srcImage.copyTo(_image_match);
    _rows=srcImage.rows;
    _count_image=1;
    _row_margine=0;
}

void MatchImage::AddImage(Mat &adding_image, int image_distance)
{
    void LineFitLeastSquares(double *data_x, double *data_y, int data_n, double *Result);//拟合直线
    int parts_both=adding_image.cols - image_distance;
    int cal_row;
    //截取大致的公共部分
    Mat parts1_use,parts2_use;
    if(_row_margine>0)
    {
        parts1_use = _image_match(Rect(_image_match.cols-parts_both,
                                         _row_margine + 10, parts_both, _rows - _row_margine - 20));
        parts2_use = adding_image(Rect(0, 0, parts_both, adding_image.rows));
        cal_row = _rows - _row_margine - 20;

    }
    else if(_row_margine<0)
    {
        parts1_use = _image_match(Rect(_image_match.cols-parts_both,
                                         10, parts_both, _rows +  _row_margine - 20));
        parts2_use = adding_image(Rect(0, 0, parts_both, adding_image.rows));
        cal_row =  _rows +  _row_margine - 20;
    }
    else
    {
        parts1_use = _image_match(Rect(_image_match.cols-parts_both,
                                         0, parts_both, _rows));
        parts2_use = adding_image(Rect(0, 0, parts_both, adding_image.rows));
        cal_row =  _rows ;
    }




    /*
     * 图像预处理
     */


    Mat part1,part2;
    GaussianBlur(parts1_use, part1, Size(3, 3), 0, 0, BORDER_DEFAULT);
    GaussianBlur(parts2_use, part2, Size(3, 3), 0, 0, BORDER_DEFAULT);
//    bilateralFilter(parts1_use,part1,10,10,10);
//    bilateralFilter(parts2_use,part2,10,10,10);

    Mat edgeCanny1,edgeCanny2;
    Canny(part1, edgeCanny1, 130, 130 * 2, 3);
    Canny(part2, edgeCanny2, 130, 130 * 2, 3);



//    Mat resizeone=Mat::zeros(1000,1200,CV_8UC1);
//    resize(edgeCanny1,resizeone,resizeone.size());
//    imshow("111",resizeone);
//    Mat resizeone3=Mat::zeros(1000,1200,CV_8UC1);
//    resize(edgeCanny2,resizeone3,resizeone3.size());
//    imshow("222",resizeone3);




    /*
     * 寻找匹配点
     */
    //存储边缘点
    Vector<Point> contoursCanny1,contoursCanny2;
    void load_edge_points(Mat &edgeCanny, Vector<Point> &contoursCanny);
    load_edge_points(edgeCanny1, contoursCanny1);
    load_edge_points(edgeCanny2, contoursCanny2);


    //将点按Y从小到大排序
    bool comp(const Point &a,const Point &b);
    sort(contoursCanny1.begin(),contoursCanny1.end(),comp);
    sort(contoursCanny2.begin(),contoursCanny2.end(),comp);

    //寻找非直线段
    int part_cols=part1.cols;
    double k_angle;
    int accumulated_times;
    int match_point1=0;
    Point seven_points1[7];//存储第8到14个点
    for(int i=20;i<part_cols-40;i=i+10)
    {

        Vector<Point> points_in;//存储这20列中的边缘点坐标
        int count_num=contoursCanny1.size();
        for(int k=0;k<count_num;k++)
        {
            int x= contoursCanny1[k].x;
            int y= contoursCanny1[k].y;

            if((y>=i)&&(y<(i+20)))
            {
                points_in.push_back(Point(x,y));
            }
        }
        //寻找非直线段
        /*
         * 将这20列中的点拟合一条直线，计算斜率，并与之前一组的
         * 拟合斜率作差，若差值大于一定值且累计同方向增加或减少两
         * 次，则认为是圆弧
         */
        int num_points=points_in.size();
        if(num_points<3)
        {
            continue;
        }
        double *points_x = new double[num_points];
        double *points_y = new double[num_points];
        double points_result[2];
        for(int k=0;k<num_points;k++)
        {
            points_x[k] = (double)points_in[k].y;
            points_y[k] = cal_row - (double)points_in[k].x;
        }

        LineFitLeastSquares(points_x, points_y, num_points, points_result);
        double k =points_result[0];
        double angle=qAtan(k);
        angle = angle*180/3.1415;
        //qDebug()<<k<<angle;
//        if(i==15)
//        {
//            k_angle=angle;
//        }
        double delta_angle = angle - k_angle;
        k_angle=angle;
        if(delta_angle>2)
        {
            accumulated_times++;
        }
        else if(delta_angle<(-2))
        {
            accumulated_times = accumulated_times-1;
        }
        else {
            accumulated_times = 0;
        }
        if(accumulated_times==2||accumulated_times==(-2))
        {
            match_point1=i;
            for(int ii=0;ii<7;ii++)
            {
                int x = points_in[ii+7].x;
                int y = points_in[ii+7].y;
                seven_points1[ii] = Point(x,y);
            }

            break;
        }
        delete[] points_x;
        delete[] points_y;

   }

    if(match_point1==0)
    {
        qDebug()<<"There is no Arc in the region";

    }
    else
    {
        //先进行大范围左右对准
        /*
         * 寻找拟合直线斜率最近的20列点
         */
        int match_point2;
        Point three_points2[3];
        double min_delta=100000;//差距最小处的差值
        int start_point = match_point1-20, end_point = match_point1+20;
        for(int i=start_point;i<end_point;i++)
        {
            Vector<Point> points_in;//存储这20列中的边缘点坐标

            int count_num=contoursCanny2.size();
            for(int k=0;k<count_num;k++)
            {
                int x= contoursCanny2[k].x;
                int y= contoursCanny2[k].y;
                if((y>=i)&&(y<(i+20)))
                {
                    points_in.push_back(Point(x,y));
                }
            }
            //计算这些点的拟合直线
            int num_points=points_in.size();
            if(num_points<3)
            {
                continue;
            }
            double *points_x = new double[num_points];
            double *points_y = new double[num_points];
            double points_result[2];
            for(int k=0;k<num_points;k++)
            {
                points_x[k] = (double)points_in[k].y;
                points_y[k] = cal_row - (double)points_in[k].x;
            }
            LineFitLeastSquares(points_x, points_y, num_points, points_result);
            double k =points_result[0];
            double angle=qAtan(k);
            angle = angle*180/3.1415;
            double delta_angle = angle-k_angle;
            if(delta_angle<0)
            {
                delta_angle=-delta_angle;
            }
            if(delta_angle<min_delta)
            {
                min_delta = delta_angle;
                match_point2 = i;
                for(int ii=0;ii<3;ii++)
                {
                    three_points2[ii] = Point(points_in[ii+9].x,points_in[ii+9].y);
                }
            }
            delete[] points_x;
            delete[] points_y;
        }
        //再进行小范围单位像素切线对准
        /*
         * 取第11个点，计算它跟左右两个像素拟合的直线，
         * 在前一张图的那7个点中，找到跟它最接近的,
         */
        double px[3],py[3],result[2];
        for(int n=0;n<3;n++)
        {
            double x = (double)three_points2[n].y;
            double y = cal_row - (double)three_points2[n].x;
            px[n]=x;
            py[n]=y;
        }
        LineFitLeastSquares(px, py, 3, result);
        double three_k = result[0];
        int min=100;
        int min_point;
        for(int ii=0;ii<5;ii++)
        {
            for(int n=ii;n<(ii+3);n++)
            {
                double x = (double)seven_points1[n].y;
                double y = cal_row - (double)seven_points1[n].x;
                px[n-ii]=x;
                py[n-ii]=y;
            }
            LineFitLeastSquares(px, py, 3, result);
            double seven_k = result[0];
            double delta_k = seven_k - three_k;
            if(delta_k<0)
            {
                delta_k = -delta_k;
            }
            if(delta_k<min)
            {
                min = delta_k;
                min_point = ii;
            }
            else if (delta_k==min)
            {
                if(ii==2)
                {
                    min_point = ii;
                }
            }
        }
        //进行拼接
        int final_x1 = seven_points1[min_point+1].x;
        int final_y1 = seven_points1[min_point+1].y;
        int final_x2 = three_points2[1].x;
        int final_y2 = three_points2[1].y;

        int delta_X = final_x1 - final_x2;
        int delta_Y = final_y1 - final_y2;
        int new_mat_rows = _image_match.rows + abs(delta_X);
        int new_mat_cols = _image_match.cols + adding_image.cols - parts_both + delta_Y;
        Mat matched_image = Mat::zeros(new_mat_rows,new_mat_cols,CV_8UC1);
        int Y1 = final_y1 + _image_match.cols - parts_both;


//        Mat resizeone3=Mat::zeros(1000,1200,CV_8UC1);
//        resize(match1,resizeone3,resizeone3.size());
//        imshow("222",resizeone3);
        if(delta_X>0)
        {
            for(int i=0;i<_image_match.rows;i++)
            {
                for(int j=0;j<=Y1;j++)
                {
                    uchar v = _image_match.at<uchar>(i,j);
                    matched_image.at<uchar>(i,j) = v;

                }
            }

            for(int i=0;i<delta_X;i++)
            {
                for(int j=Y1;j<new_mat_cols;j++)
                {
                    matched_image.at<uchar>(i,j) = 255;
                }
            }

            for(int i=delta_X;i<new_mat_rows;i++)
            {
                for(int j=Y1;j<new_mat_cols;j++)
                {
                    int x2 = i - delta_X;
                    int y2 = j - (_image_match.cols - parts_both) - delta_Y;
                    uchar v = adding_image.at<uchar>(x2,y2);
                    matched_image.at<uchar>(i,j) = v;

                }

            }

        }
        else if(delta_X<0)
        {
            int cheat = 0;
            while(cheat>(-delta_X)) cheat--;
            for(int i=0;i<(-delta_X);i++)
            {
                for(int j=0;j<=Y1;j++)
                {
                    matched_image.at<uchar>(i,j) = 255;
                }
            }
            for(int i=(-delta_X - cheat);i<(new_mat_rows - cheat);i++)
            {
                for(int j=0;j<=Y1;j++)
                {
                    uchar v = _image_match.at<uchar>(i + delta_X + cheat,j);
                    matched_image.at<uchar>(i,j) = v;

                }
            }


            for(int i=0;i<(adding_image.rows);i++)
            {
                for(int j=Y1;j<new_mat_cols;j++)
                {
                    int x2 = i ;
                    int y2 = j - (_image_match.cols - parts_both) - delta_Y;
                    uchar v = adding_image.at<uchar>(x2,y2);
                    matched_image.at<uchar>(i,j) = v;
                }
            }

        }
        else
        {
            for(int i=0;i<new_mat_rows;i++)
            {
                for(int j=0;j<=Y1;j++)
                {
                    uchar v = _image_match.at<uchar>(i,j);
                    matched_image.at<uchar>(i,j) = v;

                }
                for(int j=Y1;j<new_mat_cols;j++)
                {
                    int x2 = i - delta_X;
                    int y2 = j - (_image_match.cols - parts_both) - delta_Y;
                    uchar v = adding_image.at<uchar>(x2,y2);
                    matched_image.at<uchar>(i,j) = v;
                }

            }
        }
        matched_image.copyTo(_image_match);
        _row_margine = delta_X;
        _rows = new_mat_rows;




    }


}



Mat MatchImage::GetMatchingImage()
{
    Mat I;
    _image_match.copyTo(I);
    return I;
}

bool comp(const Point &a,const Point &b)
{
    return a.y<b.y;
}

void load_edge_points(Mat &edgeCanny, Vector<Point> &contoursCanny)
{
    for(int i=0;i<edgeCanny.rows;i++)
    {
        for(int j=0;j<edgeCanny.cols;j++)
        {
            uchar v = edgeCanny.at<uchar>(i, j) ;
            if(v!=0)
            {
                contoursCanny.push_back(Point(i,j));
            }
        }
    }
}

void LineFitLeastSquares(double *data_x, double *data_y, int data_n, double *Result)
{
    double A = 0.0;
    double B = 0.0;
    double C = 0.0;
    double D = 0.0;
    for (int i=0; i<data_n; i++)
    {
        A += data_x[i] * data_x[i];
        B += data_x[i];
        C += data_x[i] * data_y[i];
        D += data_y[i];
    }
    // 计算斜率a和截距b
    double a, b, temp = 0;
    if( temp = (data_n*A - B*B) )// 判断分母不为0
    {
        a = (data_n*C - B*D) / temp;
        b = (A*D - B*C) / temp;
    }
    else
    {
        a = 1;
        b = 0;
    }
    Result[0] = a;
    Result[1] = b;
}

