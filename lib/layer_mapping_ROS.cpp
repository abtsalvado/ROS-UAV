//
// Created by beatrizsalvado on 22-07-2018.
//

#include "layer_mapping_ROS.h"
#include "rededge_conn/conversions.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

using namespace std;
using namespace gps_common;

namespace LayerMap {

    int detect_vegetation(std::string &indice, int &first_image, cv::Mat &image_to_map, cv::Mat &imagemOriginal,
                          sensor_msgs::NavSatFix gps_coordinates,
                          grid_map::GridMap &gridMap,
                          int soil_min,
                          int soil_max,
                          int veg_min,
                          int veg_max,
                          int rocks_min,
                          int rocks_max,
                          int water_min,
                          int water_max) {
        cv::Mat frameVEG_black_and_white;
        frameVEG_black_and_white.create(480, 640, CV_8UC1);
        cv::Mat frameSOILS_black_and_white;
        frameSOILS_black_and_white.create(480, 640, CV_8UC1);
        cv::Mat frameWATER_black_and_white;
        frameWATER_black_and_white.create(480, 640, CV_8UC1);
        cv::Mat frameROCKS_black_and_white;
        frameROCKS_black_and_white.create(480, 640, CV_8UC1);



        if (indice.compare("NDVI") == 0) {
            soil_min = NDVI_SOILS_MIN;
            soil_max = NDVI_SOILS_MAX;
            veg_min = NDVI_VEG_MIN;
            veg_max = NDVI_VEG_MAX;
            rocks_min = NDVI_RCK_MIN;
            rocks_max = NDVI_RCK_MAX;
            water_min = NDVI_WATER_MIN;
            water_max = NDVI_WATER_MAX;
        } else if (indice.compare("ENDVI") == 0) {
            soil_min = ENDVI_SOILS_MIN;
            soil_max = ENDVI_SOILS_MAX;
            veg_min = ENDVI_VEG_MIN;
            veg_max = ENDVI_VEG_MAX;
            rocks_min = ENDVI_RCK_MIN;
            rocks_max = ENDVI_RCK_MAX;
            water_min = ENDVI_WATER_MIN;
            water_max = ENDVI_WATER_MAX;
        } else if (indice.compare("RDVI") == 0) {
            soil_min = RDVI_SOILS_MIN;
            soil_max = RDVI_SOILS_MAX;
            veg_min = RDVI_VEG_MIN;
            veg_max = RDVI_VEG_MAX;
            rocks_min = RDVI_RCK_MIN;
            rocks_max = RDVI_RCK_MAX;
            water_min = RDVI_WATER_MIN;
            water_max = RDVI_WATER_MAX;
        } else if (indice.compare("MSR") == 0) {
            soil_min = MSR_SOILS_MIN;
            soil_max = MSR_SOILS_MAX;
            veg_min = MSR_VEG_MIN;
            veg_max = MSR_VEG_MAX;
            rocks_min = MSR_RCK_MIN;
            rocks_max = MSR_RCK_MAX;
            water_min = MSR_WATER_MIN;
            water_max = MSR_WATER_MAX;
        } else if (indice.compare("MSAVI") == 0) {
            soil_min = MSAVI_SOILS_MIN;
            soil_max = MSAVI_SOILS_MAX;
            veg_min = MSAVI_VEG_MIN;
            veg_max = MSAVI_VEG_MAX;
            rocks_min = MSAVI_RCK_MIN;
            rocks_max = MSAVI_RCK_MAX;
            water_min = MSAVI_WATER_MIN;
            water_max = MSAVI_WATER_MAX;
        }



        std::cout << ">>  DEBUG: Detecting Vegetation Map! " << std::endl;
        std::cout << ">>  DEBUG: index! --->  " << first_image << std::endl;

        double UTMNorthing;
        double UTMEasting;
        char UTMZone;
        //LLtoUTM(gps_coordinates.latitude, gps_coordinates.longitude, UTMEasting, UTMNorthing, &UTMZone);
        LLtoUTM(gps_coordinates.latitude, gps_coordinates.longitude, UTMNorthing, UTMEasting, &UTMZone);
        ROS_INFO("UTM Robot coor(y,x):(%lf,%lf) \n", UTMNorthing, UTMEasting);

        //pos.y()=UTMNorthing;
        //pos.x()=UTMEasting

        double xDim, yDim;

        //UTMNorthing *= (10.0/RESol);
        //UTMEasting *= (10.0/RESol);

        //UTMNorthing *=10;
        //UTMEasting *= 10;
        /**Transformações**/
        static tf::TransformBroadcaster br;
        tf::Transform transform;

        /*****Altitude Variations***/
        //FieldOFView--> 48.8º --> /2 --> 24.4º
        //Img_Proportion--> 4:3 -- x:y
        //dm --> xDim * 10.0/Resol --> UTMNorthing *= (10.0/RESol); --> Retirar *=Resol do x_rot e y_rot
        //m --> xDim * 10.0--> UTMNorthing *= 10.0; --> Retirar *=Resol do x_rot e y_rot
        xDim = 2.0 * tan(Half_FOV * M_PI / 180.0) *
               gps_coordinates.altitude/*(10.0)*/;// *RESol;//*10 --> Para Mater boa reslução de Imagem e que 1 Unidade_de_Pixel--> 10cm
        yDim = Img_Proportion * xDim;
        cout << "XDIM: " << xDim << "   --- YDIM: " << yDim << endl;
        cout << "Rotacao:  " << gps_coordinates.position_covariance[0];
        //xDim = round(xDim);
        //yDim = round(yDim);
        /*if (xDim > 0 && yDim > 0) {
            cv::resize(image_to_map, image_to_map, cv::Size(xDim, yDim));
            cv::resize(frameNDVI_black_and_white, frameNDVI_black_and_white, cv::Size(xDim, yDim));
        } else
            return 0;*/

        //cout << xDim << " ---> " << yDim << endl;
        /***************************/

        if (first_image) {
            map_xDim = 100.0;
            map_yDim = 100.0;

            UTM_N_Origin = UTMNorthing;
            UTM_E_Origin = UTMEasting;

            gridMap.setGeometry(grid_map::Length(map_xDim, map_yDim), RESol);
            grid_map::Position pos;
            pos.x() = 0.0;
            pos.y() = 0.0;
            gridMap.setPosition(pos);

            gridMap.setFrameId("map");
            //gridMap[indice+"_rock"].setConstant(0.5);
            //gridMap[indice+"_soils"].setConstant(0.5);
            //gridMap[indice+"_vegetation"].setConstant(0.5);
            //gridMap[indice+"_water"].setConstant(0.5);
            //gridMap[indice+"_image_layer"].setConstant(0.5);
        }
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0); //DEFINIDO INICIALMENTE
        transform.setRotation(q);
        transform.setOrigin(tf::Vector3(UTM_E_Origin, UTM_N_Origin, 0.0));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));

        /*********TRANSFORMADA DO UAV ******/
        static tf::TransformBroadcaster br2;
        tf::Quaternion q2;
        tf::Transform transform2;

        gps_coordinates.position_covariance[0] *= ROTSYS_UAVsensor;

        transform2.setOrigin(tf::Vector3((UTMEasting - UTM_E_Origin), (UTMNorthing - UTM_N_Origin), 100.0));
        q2.setRPY(0.0, 0.0, gps_coordinates.position_covariance[0]);
        transform2.setRotation(q2);
        br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "map", "vigil_base_link"));


        //*******SE QUISER FAZER EM VARIAS TFS*****/
        /*else{
            transform.setOrigin(tf::Vector3(0, 0, 0.0));
            tf::Quaternion q;
            //q.setRPY(0, 0, 0);
            q.setRPY(0, 0, gps_coordinates.position_covariance[0]);
            transform.setRotation(q);
        }*/


        //gridMap.setGeometry(grid_map::Length(yDim,xDim),1,grid_map::Position(pos.y(), pos.x()));
        cout << " Map Pos X : " << gridMap.getPosition().x() << endl;
        cout << " Map Pos Y : " << gridMap.getPosition().y() << endl;


        /***sensor_msgs::Image img_msg;
        sensor_msgs::fillImage(img_msg, "mono8",image_to_map.rows, image_to_map.cols, image_to_map.step, image_to_map.data );

        grid_map::GridMapRosConverter::initializeFromImage(img_msg,1,gridMap);//,grid_map::Position(4360296.622286,493001.007891));

        grid_map::GridMapRosConverter::addLayerFromImage(img_msg,"image_layer",gridMap);***/


        /* ROTAÇÂO DIREITA A FUNCIONAR MAS COM BURACOS
         */
        double y_rot, x_rot, py, px;
        isInside(indice, 0.0, 0.0, xDim, yDim, gps_coordinates, gridMap, first_image, UTMNorthing, UTMEasting,
                 image_to_map);
        isInside(indice, xDim - 1.0, 0.0, xDim, yDim, gps_coordinates, gridMap, first_image, UTMNorthing, UTMEasting,
                 image_to_map);
        isInside(indice, 0.0, yDim - 1.0, xDim, yDim, gps_coordinates, gridMap, first_image, UTMNorthing, UTMEasting,
                 image_to_map);
        isInside(indice, xDim - 1.0, yDim - 1.0, xDim, yDim, gps_coordinates, gridMap, first_image, UTMNorthing,
                 UTMEasting, image_to_map);

        /*isInside(0, 0, xDim, yDim, gps_coordinates, gridMap, first_image, UTMNorthing, UTMEasting, image_to_map);
        isInside(image_to_map.cols - 1, 0, xDim, yDim, gps_coordinates, gridMap, first_image, UTMNorthing, UTMEasting, image_to_map);
        isInside(0, image_to_map.rows - 1, xDim, yDim, gps_coordinates, gridMap, first_image, UTMNorthing, UTMEasting, image_to_map);
        isInside(image_to_map.cols - 1, image_to_map.rows - 1, xDim, yDim, gps_coordinates, gridMap, first_image, UTMNorthing, UTMEasting, image_to_map);*/
        cout << "ORIGIN COORdS--> N :  " << UTM_N_Origin << " --- E : " << UTM_E_Origin << endl;
        cout << "Translation----> N :  " << N_Translation << " --- E : " << E_Translation << endl;
        cout << "ROTACAO:      " << gps_coordinates.position_covariance[0] << endl;
        int iy, ix;
        for (iy = 0; iy < image_to_map.rows; ++iy) {
            for (ix = 0; ix < image_to_map.cols; ++ix) {

                //ROTAÇÂO DIRETA
                //py = (yDim / 2 - iy);
                //px = (xDim / 2 - ix);
                double temp_x = ix;
                double temp_y = iy;
                temp_x = (temp_x * xDim / image_to_map.cols);
                temp_y = (temp_y * yDim / image_to_map.rows);
                py = (yDim / 2.0 - temp_y);
                px = (xDim / 2.0 - temp_x);
                //ix = temp_x;
                //iy = temp_y;

                //CORDENADAS PX E PY RELATIVAMENTE AO MAPA:
                double py_map = N_Translation + px;
                double px_map = E_Translation + py;


                /****************   FUNCIONAL  ****************
               x_rot = round(
                       (px_map - E_Translation) * cos((3.0 * M_PI / 2.0) - gps_coordinates.position_covariance[0]) -
                       (py_map - N_Translation) * sin((3.0 * M_PI / 2.0) - gps_coordinates.position_covariance[0]) +
                       E_Translation);
               y_rot = round(
                       (px_map - E_Translation) * sin((3.0 * M_PI / 2.0) - gps_coordinates.position_covariance[0]) +
                       (py_map - N_Translation) * cos((3.0 * M_PI / 2.0) - gps_coordinates.position_covariance[0]) +
                       N_Translation);
               ****************             ****************/


                /**************** FUNCIONAL ******************************
                 *
                 *  ROS Convention: REP-103 -->  Counter-Clockwise  (Y=0 when pointing east)   --> Useful for Trig formulas
                 *
                 *  M_PI --> Rotação de acerto da Câmara para se orientar em relação ao UAV    -->  Clockwise System -->
                 *  GPS_coordinates --> Medição de Odroid no Mission Planner --> Clockwise System
                 *
                 *********************************************/
                //std::setprecision(1);
                x_rot = //round(
                        -(px_map - E_Translation) * sin(-M_PI + gps_coordinates.position_covariance[0]) -
                        (py_map - N_Translation) * cos(-M_PI + gps_coordinates.position_covariance[0]) +
                        E_Translation;//);
                y_rot = //round(
                        (px_map - E_Translation) * cos(-M_PI + gps_coordinates.position_covariance[0]) -
                        (py_map - N_Translation) * sin(-M_PI + gps_coordinates.position_covariance[0]) +
                        N_Translation;//);

                grid_map::Position p((x_rot), (y_rot));

                //Never Paints the Back Pixels --> Usually the Black Margins

                float value_mapped;
                value_mapped = map_value(image_to_map.ptr<uchar>(iy, ix)[0], 0.0, 255.0, 1.0, 0.0);
                gridMap.atPosition(indice + "_image_layer", p) = value_mapped;

                bool paintedVEG ;
                bool paintedRCK ;
                bool paintedSOIL;
                bool paintedWTR ;
                paintedVEG = false;
                paintedRCK = false;
                paintedSOIL = false;
                paintedWTR = false;

                if ((int) image_to_map.ptr<uchar>(iy, ix)[0] > veg_min &&
                    (int) image_to_map.ptr<uchar>(iy, ix)[0] < veg_max) {
                    frameVEG_black_and_white.ptr<uchar>(iy, ix)[0] = 255;
                    /*frameROCKS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    frameSOILS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    frameWATER_black_and_white.ptr<uchar>(iy, ix)[0] = 0;*/

                    /*Only if the GridPixel is still empty*/
                    //if (gridMap.atPosition("vegetation", p) == 0.5) {
                    gridMap.atPosition(indice + "_vegetation", p) = (float) 0;
                    /*gridMap.atPosition(indice + "_soils", p) = (float) 1;
                    gridMap.atPosition(indice + "_rock", p) = (float) 1;
                    gridMap.atPosition(indice + "_water", p) = (float) 1;*/
                    paintedVEG = true;

                } //else {
                if ((int) image_to_map.ptr<uchar>(iy, ix)[0] > rocks_min &&
                    (int) image_to_map.ptr<uchar>(iy, ix)[0] < rocks_max) {
                    frameROCKS_black_and_white.ptr<uchar>(iy, ix)[0] = 255;
                    /*frameVEG_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    frameSOILS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    frameWATER_black_and_white.ptr<uchar>(iy, ix)[0] = 0;*/
                    /*Only if the GridPixel is still empty*/
                    //if (gridMap.atPosition("rock", p) == 0.5) {
                    gridMap.atPosition(indice + "_rock", p) = (float) 0;
                    /*gridMap.atPosition(indice + "_soils", p) = (float) 1;
                    gridMap.atPosition(indice + "_vegetation", p) = (float) 1;
                    gridMap.atPosition(indice + "_water", p) = (float) 1;*/
                    paintedRCK = true;

                }// else
                if ((int) image_to_map.ptr<uchar>(iy, ix)[0] > water_min &&
                    (int) image_to_map.ptr<uchar>(iy, ix)[0] < water_max) {
                    /*frameROCKS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    frameVEG_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    frameSOILS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;*/
                    frameWATER_black_and_white.ptr<uchar>(iy, ix)[0] = 255;
                    /*Only if the GridPixel is still empty*/
                    //if (gridMap.atPosition("rock", p) == 0.5) {
                    gridMap.atPosition(indice + "_water", p) = (float) 0;
                    /*gridMap.atPosition(indice + "_soils", p) = (float) 1;
                    gridMap.atPosition(indice + "_rock", p) = (float) 1;
                    gridMap.atPosition(indice + "_vegetation", p) = (float) 1;*/
                    paintedWTR = true;

                }// else
                if ((int) image_to_map.ptr<uchar>(iy, ix)[0] > soil_min/*190*/ &&
                    (int) image_to_map.ptr<uchar>(iy, ix)[0] < soil_max/*214*/) {
                    frameSOILS_black_and_white.ptr<uchar>(iy, ix)[0] = 255;
                    /*frameROCKS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    frameVEG_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    frameWATER_black_and_white.ptr<uchar>(iy, ix)[0] = 0;*/
                    /*Only if the GridPixel is still empty*/
                    //if (gridMap.atPosition("rock", p) == 0.5) {
                    gridMap.atPosition(indice + "_soils", p) = (float) 0;
                    /*gridMap.atPosition(indice + "_rock", p) = (float) 1;
                    gridMap.atPosition(indice + "_vegetation", p) = (float) 1;
                    gridMap.atPosition(indice + "_water", p) = (float) 1;*/
                    paintedSOIL = true;
                } //else {
                if (!paintedVEG) {
                    frameVEG_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    gridMap.atPosition(indice + "_vegetation", p) = (float) 1;
                }
                if(!paintedRCK){
                    frameROCKS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    gridMap.atPosition(indice + "_rock", p) = (float) 1;
                }
                if(!paintedSOIL){
                    frameSOILS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    gridMap.atPosition(indice + "_soils", p) = (float) 1;
                }
                if(!paintedWTR){
                    frameWATER_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                    gridMap.atPosition(indice + "_water", p) = (float) 1;
                }

            }
        }
        cout << "Position: X-> " << x_rot << "    Y-> " << y_rot << endl;


        std::cout << ">>  DEBUG:OUT! " << std::endl;

        /******/


        std::cout << ">>  DEBUG: Detect Vegetation Map END! " << std::endl;

        cv::imshow("image_to_map", image_to_map);
        cvWaitKey(1);
        cv::imshow("image_ORIGINAL", imagemOriginal);
        cvWaitKey(1);
        cv::imshow("Vegetation_Map", frameVEG_black_and_white);
        cvWaitKey(1);
        cv::imshow("Soils_Map", frameSOILS_black_and_white);
        cvWaitKey(1);
        cv::imshow("Rocks_Map", frameROCKS_black_and_white);
        cvWaitKey(1);
        cv::imshow("Water_Map", frameWATER_black_and_white);
        cvWaitKey(1);
        return 1;
    }


    void isInside(std::string &indice, double ix, double iy, double xDim, double yDim,
                  sensor_msgs::NavSatFix gps_coordinates,
                  grid_map::GridMap &gridMap, int first_image, double UTMNorthing, double UTMEasting,
                  cv::Mat image_to_map) {
        double y_rot, x_rot, py, px;


        N_Translation = (UTMNorthing - UTM_N_Origin);
        E_Translation = (UTMEasting - UTM_E_Origin);

        py = (yDim / 2.0 - iy);
        px = (xDim / 2.0 - ix);



        /*x_rot = round( px * sin(gps_coordinates.position_covariance[0]) -
                               py * cos(gps_coordinates.position_covariance[0]) + E_Translation);
                y_rot = round(-px * cos(gps_coordinates.position_covariance[0]) -
                               py * sin(gps_coordinates.position_covariance[0]) + N_Translation);*/


        /*x_rot = round( (px-E_Translation) * cos(M_PI) +
                       (py-N_Translation) * sin(M_PI) + E_Translation);
        y_rot = round( -(px-E_Translation) * sin(M_PI) +
                       (py-N_Translation) * cos(M_PI) + N_Translation);

        x_rot = round( (x_rot) * cos(M_PI/4) -
                       (y_rot) * sin(M_PI/4) + (x_rot - UTM_N_Origin));
        y_rot = round( (x_rot) * sin(M_PI/4) +
                       (y_rot) * cos(M_PI/4) + (y_rot - UTM_E_Origin));*/

        //CORDENADAS PX E PY RELATIVAMENTE AO MAPA:

        double py_map = N_Translation + px;
        double px_map = E_Translation + py;



        /****************   FUNCIONAL  ****************
        x_rot = round(
                (px_map - E_Translation) * cos((3.0 * M_PI / 2.0) - gps_coordinates.position_covariance[0]) -
                (py_map - N_Translation) * sin((3.0 * M_PI / 2.0) - gps_coordinates.position_covariance[0]) +
                E_Translation);
        y_rot = round(
                (px_map - E_Translation) * sin((3.0 * M_PI / 2.0) - gps_coordinates.position_covariance[0]) +
                (py_map - N_Translation) * cos((3.0 * M_PI / 2.0) - gps_coordinates.position_covariance[0]) +
                N_Translation);
        ****************             ****************/

        /**************** FUNCIONAL ******************************
         *
         *  ROS Convention: REP-103 -->  Counter-Clockwise  (Y=0 when pointing east)   --> Useful for Trig formulas
         *
         *  M_PI --> Rotação de acerto da Câmara para se orientar em relação ao UAV    -->  Clockwise System -->
         *  GPS_coordinates --> Medição de Odroid no Mission Planner --> Clockwise System
         *
         *********************************************/
        x_rot = //round(
                -(px_map - E_Translation) * sin(-M_PI + gps_coordinates.position_covariance[0]) -
                (py_map - N_Translation) * cos(-M_PI + gps_coordinates.position_covariance[0]) +
                E_Translation;//);
        y_rot = //round(
                (px_map - E_Translation) * cos(-M_PI + gps_coordinates.position_covariance[0]) -
                (py_map - N_Translation) * sin(-M_PI + gps_coordinates.position_covariance[0]) +
                N_Translation;//);

        //x_rot = (x_rot*xDim)/image_to_map.cols;
        //y_rot = (y_rot*yDim)/image_to_map.rows;
        //x_rot*=RESol;
        //y_rot*=RESol;
        /*PEQUENA: 39 deg 23' 29.45" N, 9 deg 4' 52.48" W
        GRANDE: 39 deg 23' 31.34" N, 9 deg 4' 52.92" W
                39 deg 23' 29.63" N, 9 deg 4' 51.87" W*/




        cout << "Depois de Rodado: " << x_rot << " ---> " << y_rot << endl;

        if (!gridMap.isInside(grid_map::Position(x_rot, y_rot))) {

            grid_map::GridMap tmp;
            tmp.setGeometry(grid_map::Length(map_xDim, map_yDim), RESol);
            tmp.addDataFrom(gridMap, true, true, true);
//cout <<"ABSS      " <<  abs_x << "   -->   " << abs_y << endl;

            if (!gridMap.isInside(grid_map::Position(x_rot, 0.0))) {
                map_xDim = (2.0 * abs(x_rot) + 150.0);
                cout << "Alterei X Para:" << map_xDim << endl;
            }
            if (!gridMap.isInside(grid_map::Position(0.0, y_rot))) {
                map_yDim = (2.0 * abs((y_rot)) + 150.0);
                cout << "Alterei Y Para:" << map_yDim << endl;
            }

            gridMap.setGeometry(grid_map::Length(map_xDim, map_yDim), RESol, grid_map::Position(0.0, 0.0));
            //gridMap[indice+"_rock"].setConstant(0.5);
            //gridMap[indice+"_soils"].setConstant(0.5);
            //gridMap[indice+"_vegetation"].setConstant(0.5);
            //gridMap[indice+"_water"].setConstant(0.5);
            //gridMap[indice+"_image_layer"].setConstant(0.5);
            gridMap.addDataFrom(tmp, true, true, true);
        }
        cout << "MAP DIMS: " << map_xDim << "   -->   " << map_yDim << endl;

    }

    float map_value(double x, double in_min, double in_max, double out_min, double out_max) {
        return (float) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
    }

    int test_params(cv::Mat &image_to_map,
                    int soil_min,
                    int soil_max,
                    int veg_min,
                    int veg_max,
                    int rocks_min,
                    int rocks_max,
                    int water_min,
                    int water_max) {
        cv::Mat frameVEG_black_and_white;
        frameVEG_black_and_white.create(480, 640, CV_8UC1);
        cv::Mat frameSOILS_black_and_white;
        frameSOILS_black_and_white.create(480, 640, CV_8UC1);
        cv::Mat frameWATER_black_and_white;
        frameWATER_black_and_white.create(480, 640, CV_8UC1);
        cv::Mat frameROCKS_black_and_white;
        frameROCKS_black_and_white.create(480, 640, CV_8UC1);


        double xDim, yDim;


        int iy, ix;
        for (iy = 0; iy < image_to_map.rows; ++iy) {
            for (ix = 0; ix < image_to_map.cols; ++ix) {

                bool paintedVEG ;
                bool paintedRCK ;
                bool paintedSOIL;
                bool paintedWTR ;
                paintedVEG = false;
                paintedRCK = false;
                paintedSOIL = false;
                paintedWTR = false;

                if ((int) image_to_map.ptr<uchar>(iy, ix)[0] > veg_min &&
                    (int) image_to_map.ptr<uchar>(iy, ix)[0] < veg_max) {
                    frameVEG_black_and_white.ptr<uchar>(iy, ix)[0] = 255;
                    paintedVEG = true;

                }
                if ((int) image_to_map.ptr<uchar>(iy, ix)[0] > rocks_min &&
                    (int) image_to_map.ptr<uchar>(iy, ix)[0] < rocks_max) {
                    frameROCKS_black_and_white.ptr<uchar>(iy, ix)[0] = 255;
                    paintedRCK = true;

                }
                if ((int) image_to_map.ptr<uchar>(iy, ix)[0] > water_min &&
                    (int) image_to_map.ptr<uchar>(iy, ix)[0] < water_max) {

                    frameWATER_black_and_white.ptr<uchar>(iy, ix)[0] = 255;

                    paintedWTR = true;

                }// else
                if ((int) image_to_map.ptr<uchar>(iy, ix)[0] > soil_min/*190*/ &&
                    (int) image_to_map.ptr<uchar>(iy, ix)[0] < soil_max/*214*/) {
                    frameSOILS_black_and_white.ptr<uchar>(iy, ix)[0] = 255;
                    paintedSOIL = true;
                } //else {
                if (!paintedVEG) {
                    frameVEG_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                }
                if(!paintedRCK){
                    frameROCKS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                }
                if(!paintedSOIL){
                    frameSOILS_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                }
                if(!paintedWTR){
                    frameWATER_black_and_white.ptr<uchar>(iy, ix)[0] = 0;
                }

            }
        }

        std::cout << ">>  DEBUG:OUT! " << std::endl;

        /******/


        std::cout << ">>  DEBUG: Detect Vegetation Map END! " << std::endl;

        cv::imshow("image_to_map", image_to_map);
        cvWaitKey(1);
        cv::imshow("Vegetation_Map", frameVEG_black_and_white);
        cvWaitKey(1);
        cv::imshow("Soils_Map", frameSOILS_black_and_white);
        cvWaitKey(1);
        cv::imshow("Rocks_Map", frameROCKS_black_and_white);
        cvWaitKey(1);
        cv::imshow("Water_Map", frameWATER_black_and_white);
        cvWaitKey(1);
        return 1;
    }
}


