//
// Created by beatrizsalvado on 22-07-2018.
//

#ifndef PROJECT_LAYER_MAPPING_ROS_H
#define PROJECT_LAYER_MAPPING_ROS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/pinhole_camera_model.h>
#include <iterator>
#include <numeric>
#include <time.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/NavSatFix.h>
#include "ExifTool.h"

#include <tf/transform_broadcaster.h>
#include <math.h>

//UAV Rotation System Convention
//CLOCKWISE : -1
//COUNTER-CLOCKWISE: 1
#define ROTSYS_UAVsensor (-1.0)
#define Half_FOV 24.4  // Half of Camera Field of View (Total:48.8º)
#define Img_Proportion 0.75  // Images Proportion--> 4:3    y->x = 3/4 = 0.75
#define RESol 0.1


//PARAMS Obstacles and Terrain Detection
#define NDVI_VEG_MIN 185
#define NDVI_VEG_MAX 255
#define NDVI_SOILS_MIN 70
#define NDVI_SOILS_MAX 140
#define NDVI_RCK_MIN 50
#define NDVI_RCK_MAX 105
#define NDVI_WATER_MIN 123
#define NDVI_WATER_MAX 156

#define ENDVI_VEG_MIN 170
#define ENDVI_VEG_MAX 255
#define ENDVI_SOILS_MIN 95
#define ENDVI_SOILS_MAX 147
#define ENDVI_RCK_MIN 50
#define ENDVI_RCK_MAX 120
#define ENDVI_WATER_MIN 100
#define ENDVI_WATER_MAX 156

#define RDVI_VEG_MIN 175
#define RDVI_VEG_MAX 255
#define RDVI_SOILS_MIN 85
#define RDVI_SOILS_MAX 140
#define RDVI_RCK_MIN 40
#define RDVI_RCK_MAX 108
#define RDVI_WATER_MIN 123
#define RDVI_WATER_MAX 140

#define MSR_VEG_MIN 180
#define MSR_VEG_MAX 255
#define MSR_SOILS_MIN 35
#define MSR_SOILS_MAX 125
#define MSR_RCK_MIN 50
#define MSR_RCK_MAX 105
#define MSR_WATER_MIN 115
#define MSR_WATER_MAX 160

#define MSAVI_VEG_MIN 180
#define MSAVI_VEG_MAX 255
#define MSAVI_SOILS_MIN 35
#define MSAVI_SOILS_MAX 125
#define MSAVI_RCK_MIN 40
#define MSAVI_RCK_MAX 100
#define MSAVI_WATER_MIN 122
#define MSAVI_WATER_MAX 134

namespace LayerMap
{
    //gridMap

    //grid_map::GridMap gridMap;

    //Frames
    double map_xDim, map_yDim;
    double UTM_N_Origin, UTM_E_Origin;
    double N_Translation, E_Translation;


    //int soil_min;
    //int soil_max;
    //int veg_min;
    //int veg_max;
    //int rocks_min;
    //int rocks_max;
    //int water_min;
    //int water_max;


    int detect_vegetation(std::string &indice, int &first_image, cv::Mat &image_to_map, cv::Mat &imagemOriginal, sensor_msgs::NavSatFix gps_coordinates,
                          grid_map::GridMap &gridMap,
                          int soil_min,
                          int soil_max,
                          int veg_min,
                          int veg_max,
                          int rocks_min,
                          int rocks_max,
                          int water_min,
                          int water_max);
    void isInside(std::string &indice, double ix, double iy, double xDim, double yDim, sensor_msgs::NavSatFix gps_coordinates,
                  grid_map::GridMap &gridMap, int first_image, double UTMNorthing, double UTMEasting, cv::Mat image_to_map);
    float map_value(double x, double in_min, double in_max, double out_min, double out_max);

    int test_params(cv::Mat &image_to_map,
                    int soil_min,
                    int soil_max,
                    int veg_min,
                    int veg_max,
                    int rocks_min,
                    int rocks_max,
                    int water_min,
                    int water_max);
}

#endif //PROJECT_LAYER_MAPPING_ROS_H
