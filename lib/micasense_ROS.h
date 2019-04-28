//
// Created by beatrizsalvado on 29-05-2018.
//

#ifndef REDEDGE_CONN_MICASENSE_ROS_H
#define REDEDGE_CONN_MICASENSE_ROS_H

#include <iostream>

#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>


#include <string>
#include <math.h>
#include <vector>
#include <highgui.h>

#include "ExifTool.h"

#include "opencv2/xfeatures2d.hpp"


/****** Choose File Folder Includes ***/
#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>

namespace fs = ::boost::filesystem;
#include <algorithm>    // std::sort

#include <ros/package.h>

#define HESS 600
#define DIST 100
#define MUX 9
/**************************************/


using namespace std;
using namespace cv::xfeatures2d;


namespace Micasense
{
    cv::Mat null_mat;



    //VARIABLES: raw_image_to_radiance
    //double *Yi;
    //double *ri;
    //double *vignettei;


    int indx_Mouse;
    int iy, ix;
    cv::Point UpLeft_clicked;
    cv::Point LowtRight_clicked;

    ExifTool *et = new ExifTool();
    //ExifTool *et2 = new ExifTool();

    void get_all(const fs::path& root, const string& ext, vector<fs::path>& ret);
    void CallBackFunc(int event, int x, int y, int flags, void* userdata);
    double calib_init(std::stringstream &img_path, cv::Mat &imageRaw, cv::Mat &radianceImage, cv::Mat &L, cv::Mat &V, cv::Mat &R);
    void raw_image_to_radiance(std::stringstream &img_path, cv::Mat &imageRaw, cv::Mat &radianceImage, cv::Mat &L, cv::Mat &V, cv::Mat &R);
    cv::Mat correct_lens_distortion(cv::Mat &refl_image, std::stringstream &img_path);

    void vignette_map(int xDim, int yDim, double xVignette, double yVignette, std::vector<double> vignettePolyList, cv::Mat& Y, cv::Mat& vignette);

    void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv,
                  cv::Mat1i &X, cv::Mat1i &Y);
    void meshgridTest(const cv::Range &xgv, const cv::Range &ygv,
                      cv::Mat1i &X, cv::Mat1i &Y);


    double dist(CvPoint a, CvPoint b);
    double getMaxDisFromCorners(const cv::Size& imgSize, const cv::Point& center);
    void generateGradient(cv::Mat& mask);

    cv::Point panelHomography(cv::Mat img_scene, int &cropped_heigth, int &cropped_width, cv::RotatedRect &rect);
    int TestpanelHomography();
}

#endif //REDEDGE_CONN_MICASENSE_ROS_H
