//
// Created by beatrizsalvado on 28-05-2018.
//

#include "micasense_ROS.h"

#include <iomanip>

int xDim;
int yDim;
string bandName;


using namespace std;
using namespace cv;

namespace Micasense {

    void get_all(const fs::path &root, const string &ext, vector<fs::path> &ret) {

        if (!fs::exists(root) || !fs::is_directory(root)) return;

        fs::recursive_directory_iterator it(root);
        fs::recursive_directory_iterator endit;

        while (it != endit) {
            if (fs::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
            ++it;
        }
        sort(ret.begin(), ret.end());
    }

    void CallBackFunc(int event, int x, int y, int flags, void *userdata) {
        if (event == EVENT_LBUTTONDOWN) {
            cout << "Left mouse button is clicked - position (" << x << ", " << y << ")" << endl;
            if (indx_Mouse == 0) {
                UpLeft_clicked.x = x;
                UpLeft_clicked.y = y;
            } else {
                LowtRight_clicked.x = x;
                LowtRight_clicked.y = y;
            }
        }
    }

/*************************** calib_init **********
 *
 * @param img_path
 * @param image_Raw
 * @param radianceImage
 * @param L
 * @param V
 * @param R
 * @return radianceToReflectance
 */
    double calib_init(std::stringstream &img_path, cv::Mat &image_Raw, cv::Mat &radianceImage, cv::Mat &L, cv::Mat &V,
                      cv::Mat &R) {
        int height;
        int width;
        cv::Mat panelRegion;
        RotatedRect rect;

        raw_image_to_radiance(img_path, image_Raw, radianceImage, L, V, R);

        cv::Mat markedImg = radianceImage.clone();
//        cv::Mat show;
//        cv::normalize(image_Raw, show, 0, 1, cv::NORM_MINMAX);
//
//        for (indx_Mouse = 0; indx_Mouse < 2; indx_Mouse++) {
//            if (indx_Mouse == 0) {
//                cout << " >> Select Up Left Corner" << endl;
//            } else
//                cout << " >> Select Low Right Corner" << endl;
//            namedWindow("MarkSquare WIN", 1);
//            setMouseCallback("MarkSquare WIN", CallBackFunc, NULL);
//            imshow("MarkSquare WIN", show);
//            waitKey(0);
//
//
//            if (indx_Mouse == 1) {
//                cout << " --(micasense_ros) DEBUG: Up LefT:  " << UpLeft_clicked << endl;
//                cout << " --(micasense_ros) DEBUG: Low RighT:  " << LowtRight_clicked << endl;
//
//                height = LowtRight_clicked.y - UpLeft_clicked.y;
//                width = LowtRight_clicked.x - UpLeft_clicked.x;
//                if (height < 0 || width < 0) {
//                    indx_Mouse = -1;
//                    cout << " --(!) !!!  Please Make Sure you Select The Square Corners correctly  !!!  " << endl;
//                }
//            }
//        }


        cv::Mat img;
        img = cv::imread(img_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
        cv::Point ULCrop_Panel;
        UpLeft_clicked = panelHomography(img, height, width, rect);
        /*markedImg = markedImg(boundRect);*/

        Rect boundRect = rect.boundingRect();

        int area =  boundRect.area();
        std::cout << " Area: " << area << std::endl;
        float angle = rect.angle;
        Size2f rect_size = rect.size;
        if (rect.angle < -45.) {
            angle += 90.0;
            swap(rect_size.width, rect_size.height);
        }
        Mat Mrect, rotated, cropped;
        Mrect = getRotationMatrix2D(rect.center, angle, 1.0);
        warpAffine(markedImg, rotated, Mrect, markedImg.size(), INTER_CUBIC);
        rotated *= 255.0;
        rotated.convertTo(rotated, CV_8UC1);
        getRectSubPix(rotated, rect_size, rect.center, markedImg);
        markedImg.convertTo(markedImg, CV_64F, 1.0 / 255.0);

        cout << "UpLEFT X: " << UpLeft_clicked.x << "    ***   UpLEFT Y: " << UpLeft_clicked.y << endl;
        cout << "Width: " << width << "    ***   Heigth: " << height << endl;
        panelRegion = markedImg(Rect(UpLeft_clicked.x, UpLeft_clicked.y, width, height));
        /*imshow("PanelRegion", panelRegion);
        cvWaitKey(1);*/

        //resize(markedImg,markedImg,Size(1280,960));
        //cv::resizeWindow("MarkedImage", markedImg.size().width/2, markedImg.size().height/2);

        /*cv::namedWindow("MarkedImage", cv::WINDOW_NORMAL);
        cv::resizeWindow("MarkedImage", 640, 480);
        imshow("MarkedImage", markedImg);
        waitKey(1);*/

        Scalar tempVal = mean(panelRegion);
        double meanRadiance = tempVal.val[0];

        //double meanRadiance = 29.701;

//        cout   << "PanelRegion:  " << endl
//               << panelRegion.at<double>(0, 0) <<endl
//               << panelRegion.at<double>(10, 10)<<endl
//               << panelRegion.at<double>(20, 20)<<endl
//               << panelRegion.at<double>(30, 30)<<endl
//               << panelRegion.at<double>(height-1,width-1)<<endl;


        std::string path;
        path = "src/rededge_conn/calib&align/real_cam_YAMLs/panelCalibration.yml";
        FileStorage fs2(path, FileStorage::READ);

        double panelReflectance;
        double radianceToReflectance;
        fs2[bandName] >> panelReflectance;

        /*std::cout << "Blue: " << blue << endl
                  << "Green: " << green << endl
                  << "Red: " << red << endl
                  << "Rededge: " << rededge << endl
                  << "Nir: " << nir << endl;*/

        //            fs2.release();

        radianceToReflectance = panelReflectance / meanRadiance;
        cout << " --(micasense_ros) DEBUG: Mean Value -->" << meanRadiance << endl
             << " --(micasense_ros) DEBUG: PanelReflectance -->" << panelReflectance << endl
             << " --(micasense_ros) DEBUG: Radiancetoreflectance -->" << radianceToReflectance << endl;

        cv::Mat reflectanceImage(yDim, xDim, CV_64F);
        reflectanceImage = radianceImage * radianceToReflectance;

        //reflectanceImage *= 255;
        //reflectanceImage.convertTo(reflectanceImage, CV_8UC1);
//        cout   << "Radiance to reflectance:"<< endl
//               << reflectanceImage.at<double>(0, 0) << endl
//               << reflectanceImage.at<double>(10, 10)<<endl
//               << reflectanceImage.at<double>(20, 20)<<endl
//               << reflectanceImage.at<double>(30, 30)<<endl
//               << reflectanceImage.at<double>(959, 1279)<<endl;

        cv::Mat panelRegionRefl;
        cv::Mat panelRegionReflBlur;
        panelRegionRefl = reflectanceImage(Rect(UpLeft_clicked.x, UpLeft_clicked.y, width, height));
//        cout   << "panelRegionRefl:"<< endl
//               << panelRegionRefl.at<double>(0, 0) << endl
//               << panelRegionRefl.at<double>(10, 10)<<endl
//               << panelRegionRefl.at<double>(20, 20)<<endl
//               << panelRegionRefl.at<double>(30, 30)<<endl
//               << panelRegionRefl.at<double>(height-1,width-1)<<endl;

        GaussianBlur(panelRegionRefl, panelRegionReflBlur, cv::Size(55, 55), 5);
//        cout   << "GaussianBlur:"<< endl
//               //                   << panelRegionReflBlur.at<double>(0, 0) << endl
//               //                   << panelRegionReflBlur.at<double>(10, 10)<<endl
//               //                   << panelRegionReflBlur.at<double>(20, 20)<<endl
//
//               << panelRegionReflBlur.at<double>(30, 30)<<endl
//               << panelRegionReflBlur.at<double>(54, 54) << endl
//               << panelRegionReflBlur.at<double>(55, 55) << endl
//               << panelRegionReflBlur.at<double>(57, 57) << endl
//               << panelRegionReflBlur.at<double>(60, 60) << endl
//               << panelRegionReflBlur.at<double>(70, 70)<<endl
//               << panelRegionReflBlur.at<double>(80, 80)<<endl
//               << panelRegionReflBlur.at<double>(90, 90)<<endl
//               << panelRegionReflBlur.at<double>(100,100)<<endl;
//                    << panelRegionReflBlur.at<double>(width-1,height-1)<<endl;







/*************************************************************/

        double min_Refl, max_Refl;
        cv::minMaxLoc(panelRegionRefl, &min_Refl, &max_Refl);

        cv::Scalar meanRefl, stddev;
        meanStdDev(panelRegionRefl, meanRefl, stddev);
        cout << " --(micasense_ros) DEBUG: Min Reflectance in panel region: " << min_Refl << endl;
        cout << " --(micasense_ros) DEBUG: Max Reflectance in panel region: " << max_Refl << endl;
        cout << " --(micasense_ros) DEBUG: Mean Reflectance in panel region: " << meanRefl[0] << endl;
        cout << " --(micasense_ros) DEBUG: Standard deviation in region: " << stddev[0] << endl;

        /**************SCALING******/
        /*cv::namedWindow("GaussianBlur", cv::WINDOW_NORMAL);
        Mat output = panelRegionReflBlur.clone();
        output *= 255.0;
        double min_Gauss, max_Gauss;
        cv::minMaxLoc(output, &min_Gauss, &max_Gauss);
        for (int iy = 0; iy < output.rows; ++iy) {
            for (int ix = 0; ix < output.cols; ++ix) {
                output.at<double>(ix, iy) = (output.at<double>(ix, iy)-min_Gauss)/(max_Gauss-min_Gauss);
            }
        }
        output.convertTo(output, CV_8UC1);

        cv::applyColorMap(output, output, cv::COLORMAP_RAINBOW);

        cv::resizeWindow("GaussianBlur", 640,480);
        imshow("GaussianBlur", output);
        waitKey(1);*/
        /********************************/

        //correct_lens_distortion(reflectanceImage);
        return radianceToReflectance;
    }

    /******* raw_image_to_radiance ****
     *
     * @param img_path
     * @param imageRaw
     * @param radianceImage
     * @param L
     * @param V
     * @param R
     */
    void raw_image_to_radiance(std::stringstream &img_path, cv::Mat &imageRaw, cv::Mat &radianceImage, cv::Mat &L,
                               cv::Mat &V, cv::Mat &R) {
//imageRaw=imageRaw.data;
        cout << std::fixed << std::setprecision(13) << endl;

        xDim = imageRaw.cols;
        yDim = imageRaw.rows;

        double xVignette;
        double yVignette;
        std::vector<double> vignettePolyList;

        // radiometric sensitivity
        //std::vector<char * > val;

        //ExifTool *et = new ExifTool(); <--
        // read metadata from the image
        //TagInfo *info = et->ImageInfo("../thesis_ws/src/blue.tif");
        double a1 = 0.0, a2 = 0.0, a3 = 0.0;
        double val = 0.0;
        double num = 0.0, den = 0.0;
        double exposureTime = 0.0;
        double darkLevel = 0.0;
        double bitsPerPixel = 0.0;
        double gain = 0.0;

        /* std::string path;
         //path = "src/rededge_conn/calib&align/" + BandAlignmentOF + "_rededge.yml";
         path = "src/rededge_conn/calib&align/real_cam_YAMLs/meta_d.yml";
         cout     << "entreiii" << endl;

         FileStorage fs2(path, FileStorage::READ);
         std::string info_r;
         fs2["RadiometricCalibration"] >> info_r;
         sscanf(info_r.c_str(), "%lf,%lf,%lf", &a1, &a2, &a3);

         fs2["BlackLevel"] >> info_r;
         std::vector<float> black_levels;
         std::stringstream iss(info_r);
         while (iss >> val)
             black_levels.push_back(val);
         darkLevel = accumulate(black_levels.begin(), black_levels.end(), 0.0) / black_levels.size();

         fs2["ExposureTime"] >> info_r;
         sscanf(info_r.c_str(), "%lf/%lf", &num, &den);
         exposureTime = num / den;

         fs2["ISOSpeed"] >> info_r;
         sscanf(info_r.c_str(), "%lf", &gain);
         gain = gain / 100;


         fs2["VignettingCenter"] >> info_r;
         sscanf(info_r.c_str(), "%lf, %lf", &xVignette, &yVignette);

         fs2["VignettingPolynomial"] >> info_r;
         std::stringstream iss2(info_r);
         while (iss2 >> val) {
             vignettePolyList.push_back(val);
             if (iss2.peek() == ',')
                 iss2.ignore();
         }
         std::reverse(vignettePolyList.begin(), vignettePolyList.end());
         vignettePolyList.push_back(1.0);

         fs2["BitsPerSample"] >> info_r;
         sscanf(info_r.c_str(), "%lf", &bitsPerPixel);

         fs2["BandName"] >> info_r;
         bandName = info_r;

         cout     << "entreiii" << bitsPerPixel << endl;

         fs2.release();*/
/********TAG EXFIFTOOL ****/


        try {
            std::string str_stream = img_path.str();

            TagInfo *info = et->ImageInfo(
                    str_stream.c_str());

            if (info) {

                // print returned information
                for (TagInfo *i = info; i; i = i->next) {
                    //cout << i->name << " = " << i->value << endl;
                    string index_name = (string) i->name;
                    if (index_name.compare("RadiometricCalibration") == 0) {
                        sscanf(i->value, "%lf,%lf,%lf", &a1, &a2, &a3);
                    }
                    if (index_name.compare("BlackLevel") == 0) {
                        std::vector<float> black_levels;
                        std::stringstream iss(i->value);
                        while (iss >> val)
                            black_levels.push_back(val);
                        darkLevel = accumulate(black_levels.begin(), black_levels.end(), 0.0) / black_levels.size();
                    }
                    if (index_name.compare("ExposureTime") == 0) {
                        sscanf(i->value, "%lf/%lf", &num, &den);
                        exposureTime = num / den;
                        //exposureTime = 0.0009675; //----//PYTHON Value


                    }
                    if (index_name.compare("ISOSpeed") == 0) {
                        sscanf(i->value, "%lf", &gain);
                        gain = gain / 100.0;
                    }
                    if (index_name.compare("VignettingCenter") == 0) {
                        sscanf(i->value, "%lf, %lf", &xVignette, &yVignette);
                    }
                    if (index_name.compare("VignettingPolynomial") == 0) {
                        std::stringstream iss(i->value);
                        while (iss >> val) {
                            vignettePolyList.push_back(val);
                            if (iss.peek() == ',')
                                iss.ignore();
                        }
                        std::reverse(vignettePolyList.begin(), vignettePolyList.end());
                        vignettePolyList.push_back(1.0);
                    }
                    if (index_name.compare("BitsPerSample") == 0) {
                        sscanf(i->value, "%lf", &bitsPerPixel);
                    }
                    if (index_name.compare("BandName") == 0) {
                        bandName = i->value;
                    }


                }
                // we are responsible for deleting the information when done
                delete info;
                cout << " --(micasense_ros) DEBUG: exposureTime:   " << exposureTime << "NUM/DEN: "<< num <<" ; "<< den<< endl;
                cout << " --(micasense_ros) DEBUG: gain:   " << gain << endl;

                /*apply image correction methods to raw image
                # step 1 - row gradient correction, vignette & radiometric calibration:
                # compute the vignette map image
                 */



                /*cv::Mat maskImg(imageRaw.size(), CV_64F);

                generateGradient(imageRaw);

                cv::Mat gradient;
                cv::normalize(maskImg, gradient, 0, 255, CV_MINMAX);
                cv::imwrite("gradient.png", gradient);

                /*cv::Mat labImg(imageRaw.size(), CV_8UC3);
                cv::cvtColor(imageRaw, labImg, CV_BGR2Lab);

                for (int row = 0; row < labImg.size().height; row++)
                {
                    for (int col = 0; col < labImg.size().width; col++)
                    {
                        cv::Vec3b value = labImg.at<cv::Vec3b>(row, col);
                        value.val[0] *= maskImg.at<double>(row, col);
                        labImg.at<cv::Vec3b>(row, col) =  value;
                    }
                }

                cv::Mat output;
                cv::cvtColor(labImg, output, CV_Lab2BGR);
                //cv::imwrite("vignette.png", output);

                cv::namedWindow("Vignette", cv::WINDOW_NORMAL);
                cv::resizeWindow("Vignette", output.size().width/2, output.size().height/2);
                cv::imshow("Vignette", output);
                cv::waitKey();*/

/***/
            } else if (et->LastComplete() <= 0) {
                cerr << "Error executing exiftool!" << endl;
            }
            // print exiftool stderr messages
            char *err = et->GetError();
            if (err) {
                cout << err;
            }
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            cv::Mat null_mat = Mat(1, 1, CV_64F, double(0));

        }



/****/
/******END EXIFTOOL***************/

        //delete et; //<--

        cv::Mat Y(yDim, xDim, CV_64F);

        double bitDepthMax = pow(2, bitsPerPixel);
        /*vignette = */vignette_map(xDim, yDim, xVignette, yVignette, vignettePolyList, Y, V);
        //V, x, y = vignette_map(meta, xDim, yDim);

        /*double *Ri;
        double *Yi;
        double *Vi;
        double *Li;
        size_t step = R.step;*/

        for (iy = 0; iy < yDim; ++iy) {
            // Ri = R.ptr<double>(iy);
            for (ix = 0; ix < xDim; ++ix) {
                /***row gradient correction***/
                /*Ri[ix]  =
                        1.0 / (1.0 + a2 * Y.ptr<double>(iy,ix)[0]/ exposureTime - a3 * Y.ptr<double>(iy,ix)[0]);*/
                R.ptr<double>(iy, ix)[0] =
                        1.0 / (1.0 + a2 * Y.ptr<double>(iy, ix)[0] / exposureTime - a3 * Y.ptr<double>(iy, ix)[0]);

                //cout << "Olaola   -->" <<ix <<";" << iy <<"-->" <<(int)imageRaw.at<uchar>(iy,ix) << endl;

                /***subtract the dark level and adjust for vignette and row gradient***/
                L.ptr<double>(iy, ix)[0] = V.ptr<double>(iy, ix)[0] * R.ptr<double>(iy, ix)[0] *
                                           ((double) imageRaw.ptr<float>(iy, ix)[0] - darkLevel);

                /***Floor any negative radiances to zero (can happend due to noise around blackLevel)***/
                if (L.ptr<double>(iy, ix)[0] < 0) {
                    L.ptr<double>(iy, ix)[0] = 0.0;
                }

                /**************************************************************************
                apply the radiometric calibration - i.e. scale by the gain-exposure product and
                multiply with the radiometric calibration coefficient
                need to normalize by 2^16 for 16 bit images
                because coefficients are scaled to work with input values of max 1.0
                ******************************************************/
                radianceImage.ptr<double>(iy, ix)[0] =
                        L.ptr<double>(iy, ix)[0] / (gain * exposureTime) * a1 / bitDepthMax;
            }
        }

//        cout << "Y:   " << endl
//             << Y.ptr<double>(0, 0)[0] << endl
//             << Y.ptr<double>(10, 10)[0] << endl
//             << Y.ptr<double>(20, 20)[0] << endl
//             << Y.ptr<double>(30, 30)[0] << endl
//             << Y.ptr<double>(959, 1279)[0] << endl;
//
//        cout << "Y:   " << endl
//             << Y.at<double>(0, 0) << endl
//             << Y.at<double>(10, 10) << endl
//             << Y.at<double>(20, 20) << endl
//             << Y.at<double>(30, 30) << endl
//             << Y.at<double>(959, 1279) << endl;
//
//        cout << "R:   " << endl
//             << R.at<double>(0, 0) << endl
//             << R.at<double>(10, 10) << endl
//             << R.at<double>(20, 20) << endl
//             << R.at<double>(30, 30) << endl
//             << R.at<double>(959, 1279) << endl;
//
//        cout << "vignette:   " << endl
//             << V.at<double>(0, 0) << endl
//             << V.at<double>(10, 10) << endl
//             << V.at<double>(20, 20) << endl
//             << V.at<double>(30, 30) << endl
//             << V.at<double>(959, 1279) << endl;
//
//        cout << "L:   " << endl
//             << L.at<double>(0, 0) << endl
//             << L.at<double>(10, 10) << endl
//             << L.at<double>(20, 20) << endl
//             << L.at<double>(30, 30) << endl
//             << L.at<double>(959, 1279) << endl;
//
//        cout << "RadianceImage:   " << endl << radianceImage.at<double>(0, 0) << endl
//             << radianceImage.at<double>(10, 10) << endl
//             << radianceImage.at<double>(20, 20) << endl
//             << radianceImage.at<double>(30, 30) << endl
//             << radianceImage.at<double>(959, 1279) << endl;
    }

    /***********/



    /*radianceImage *= 255;
    radianceImage.convertTo(radianceImage, CV_8UC1);

    cv::applyColorMap(lol, lol, cv::COLORMAP_JET);

    cv::namedWindow("Radiance Image", CV_WINDOW_FULLSCREEN);
    imshow("Radiance Image", radianceImage);
    cv::waitKey(1);*/

    /*try {
        cv::imwrite("src/Cradimage.tif", lol);
    }catch(cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }*/

    /************ correct_lens_distortion ************
     *
     * @param refl_image
     * @param img_path
     * @return undistortedImage
     *********/
    cv::Mat correct_lens_distortion(cv::Mat &refl_image, std::stringstream &img_path) {

        int w = refl_image.cols;
        int h = refl_image.rows;


        float fx = 0.0, fy = 0.0, cX = 0.0, cY = 0.0, xPP = 0.0, yPP = 0.0;
        float FocalPlaneXResolution = 0.0, FocalPlaneYResolution = 0.0;
        float focal_length_px = 0.0, focal_length_mm = 0.0;
        std::vector<float> distortionParameters;
        string units;
        float val = 0.0;
        try {
//            TagInfo *info = et->ImageInfo(
//                    "/home/beatrizsalvado/Desktop/DESK/Mission_1_DataSet/28_05_2018/0016SET/000/IMG_0004_4.tif");
            std::string str_stream = img_path.str();

            TagInfo *info = et->ImageInfo(
                    str_stream.c_str());
            if (info) {

                // print returned information
                for (TagInfo *i = info; i; i = i->next) {
                    //cout << i->name << " = " << i->value << endl;
                    string index_name = (string) i->name;
                    if (index_name.compare("FocalPlaneXResolution") == 0) {
                        sscanf(i->value, "%f", &FocalPlaneXResolution);
                    }
                    if (index_name.compare("FocalPlaneYResolution") == 0) {
                        sscanf(i->value, "%f", &FocalPlaneYResolution);

                    }
                    if (index_name.compare("PrincipalPoint") == 0) {
                        sscanf(i->value, "%f, %f", &xPP, &yPP);

                    }
                    if (index_name.compare("PerspectiveFocalLength") == 0) {
                        sscanf(i->value, "%f", &focal_length_mm);

                    }
                    if (index_name.compare("PerspectiveFocalLengthUnits") == 0) {
                        units = i->value;

                    }
                    if (index_name.compare("PerspectiveDistortion") == 0) {
                        std::stringstream iss(i->value);
                        while (iss >> val) {
                            distortionParameters.push_back(val);
                            if (iss.peek() == ',')
                                iss.ignore();
                        }
                        //std::reverse(distortionParameters.begin(), distortionParameters.end());
                    }
                }
                if (units.compare("mm") != 0) {
                    focal_length_mm = focal_length_mm / FocalPlaneXResolution;
                }
                // we are responsible for deleting the information when done
                delete info;
            } else if (et->LastComplete() <= 0) {
                cerr << "Error executing exiftool!" << endl;
            }
            // print exiftool stderr messages
            char *err = et->GetError();
            if (err) {
                cout << err;
            }
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            cv::Mat null_mat = Mat(1, 1, CV_32F, float(0));
        }

        cX = xPP * FocalPlaneXResolution;
        cY = yPP * FocalPlaneYResolution;
        fx = focal_length_mm * FocalPlaneXResolution;
        fy = focal_length_mm * FocalPlaneYResolution;

//        cout << focal_length_mm << endl;
//        cout << FocalPlaneXResolution << endl;


        cv::Mat cam_mat = Mat(3, 3, CV_32F, float(0));

        cam_mat.at<float>(0, 0) = fx;
        cam_mat.at<float>(1, 1) = fy;
        cam_mat.at<float>(2, 2) = 1.0;
        cam_mat.at<float>(0, 2) = cX;
        cam_mat.at<float>(1, 2) = cY;

//        cout<< "Cam_Mat" <<endl;
//        for (int iy = 0; iy < cam_mat.rows; ++iy) {
//            for (int ix = 0; ix < cam_mat.cols; ++ix) {
//                cout << cam_mat.at<float>(iy, ix)<< endl;
//            }
//        }
//        cout<< "distortionParameters" <<endl;
//        for(std::vector<float>::const_iterator i = distortionParameters.begin();i!=distortionParameters.end();++i)
//            cout << *i << endl;
        std::vector<float> dist_coeffs;
        dist_coeffs.emplace_back(distortionParameters.at(0));
        dist_coeffs.emplace_back(distortionParameters.at(1));
        dist_coeffs.emplace_back(distortionParameters.at(3));
        dist_coeffs.emplace_back(distortionParameters.at(4));
        dist_coeffs.emplace_back(distortionParameters.at(2));
        //dist_coeffs.emplace_back(distortionParameters.at(0),distortionParameters.at(1),distortionParameters.at(3),distortionParameters.at(4),distortionParameters.at(2));
        // distortionParameters[[0, 1, 3, 4, 2]];
//        cout<< "dist_coeffs" <<endl;
//        for(std::vector<float>::const_iterator i = dist_coeffs.begin();i!=dist_coeffs.end();++i)
//            cout << *i << endl;

        cv::Mat new_cam_mat(3, 3, CV_32F);
        cv::Size size_mat(xDim, yDim);
        new_cam_mat = cv::getOptimalNewCameraMatrix(cam_mat, dist_coeffs, size_mat, 1);
        //cv::Mat new_cam_mat = cv::getOptimalNewCameraMatrix(cam_mat,dist_coeffs, size_mat,1);

//        cout<< "new_cam_mat" <<endl;
//        for (int iy = 0; iy < new_cam_mat.rows; ++iy) {
//            for (int ix = 0; ix < new_cam_mat.cols; ++ix) {
//                cout << new_cam_mat.at<float>(iy, ix)<< endl;
//            }
//        }

        cv::Mat identity_mat(3, 3, CV_32F, float(0));
        for (int i = 0; i < 3; i++)
            identity_mat.at<float>(i, i) = 1.0;
//        cout<< "identity_mat" <<endl;
//        for (int iy = 0; iy < 3; ++iy) {
//            for (int ix = 0; ix < 3; ++ix) {
//                cout << identity_mat.at<float>(iy,ix)<< endl;
//            }
//        }


        cv::Mat map1(yDim, xDim, CV_32F);
        cv::Mat map2(yDim, xDim, CV_32F);

        initUndistortRectifyMap(cam_mat, dist_coeffs, identity_mat, new_cam_mat, size_mat, CV_32F, map1, map2);
//
//
//        cout<< "w" <<endl;
//        cout<< size_mat.width <<endl;
//        cout<< "h" <<endl;
//        cout<< size_mat.height <<endl;
//        cout<< "Map1" <<endl;
//        cout << "map1:   " << endl
//             << map1.at<float>(0, 0) << endl
//             << map1.at<float>(10, 10) << endl
//             << map1.at<float>(20, 20) << endl
//             << map1.at<float>(30, 30) << endl
//             << map1.at<float>(959, 1279) << endl;
//        cout<< "Map2" <<endl;
//        cout << "map2:   " << endl
//             << map2.at<float>(0, 0) << endl
//             << map2.at<float>(10, 10) << endl
//             << map2.at<float>(20, 20) << endl
//             << map2.at<float>(30, 30) << endl
//             << map2.at<float>(959, 1279) << endl;

        cv::Mat undistortedImage(yDim, xDim, CV_64F);
        //refl_image.convertTo(refl_image, CV_32F);
        remap(refl_image, undistortedImage, map1, map2, INTER_LINEAR);


        //       imshow("undistortedImage", undistortedImage);
        //       waitKey(1);

        return undistortedImage;
    }


/***************** vignette_map ************
 *
 * @param xDim
 * @param yDim
 * @param xVignette
 * @param yVignette
 * @param vignettePolyList
 * @param Y
 * @param vignette
 ******************************************/
    void vignette_map(int xDim, int yDim, double xVignette, double yVignette, std::vector<double> vignettePolyList,
                      cv::Mat &Y,
                      cv::Mat &vignette) {

        cv::Mat r(yDim, xDim, CV_64F);
        //int iy, ix;

        ros::Time time_initial = ros::Time::now();
        double powy;
/*
        double *Yi;
        double *ri;
        double *vignettei;*/
        for (iy = 0; iy < yDim; ++iy) {
            double *Yi = Y.ptr<double>(iy);
            double *ri = r.ptr<double>(iy);
            double *vignettei = vignette.ptr<double>(iy);
            powy = pow((yVignette - iy), 2);
            for (ix = 0; ix < xDim; ++ix) {
                Yi[ix] = iy;

                //ri[ix] = sqrt(pow((xVignette - ix), 2) + pow((yVignette - iy), 2));
                ri[ix] = sqrt(pow((xVignette - ix), 2) + powy);


                vignettei[ix] = 1. / (vignettePolyList[0] * pow(ri[ix], 6) +
                                      vignettePolyList[1] * pow(ri[ix], 5) +
                                      vignettePolyList[2] * pow(ri[ix], 4) +
                                      vignettePolyList[3] * pow(ri[ix], 3) +
                                      vignettePolyList[4] * pow(ri[ix], 2) +
                                      vignettePolyList[5] * ri[ix] +
                                      vignettePolyList[6]);

                /*(vignettePolyList[0] * pow(ri[ix], 6) +
                                      vignettePolyList[1] * pow(ri[ix], 5) +
                                      vignettePolyList[2] * pow(ri[ix], 4) +
                                      vignettePolyList[3] * pow(ri[ix], 3) +
                                      vignettePolyList[4] * pow(ri[ix], 2) +
                                      vignettePolyList[5] *     ri[ix] +
                                      vignettePolyList[6]);*/
            }
        }
        ros::Time time_final = ros::Time::now();
        std::cout << " --(micasense_ros) DEBUG: ImgReflectance&Lens_Corrections ----- TIME STAMP:  "
                  << time_final - time_initial << std::endl;

        /*ros::Time time_initial = ros::Time::now();
        for (iy = 0; iy < yDim; ++iy) {

            for (ix = 0; ix < xDim; ++ix) {
                Y.ptr<double>(iy, ix)[0] = iy;

                r.ptr<double>(iy, ix)[0] = sqrt(pow((xVignette - ix), 2) + pow((yVignette - iy), 2));


                vignette.ptr<double>(iy, ix)[0] = 1. / (vignettePolyList[0] * pow(r.ptr<double>(iy, ix)[0], 6) +
                                                        vignettePolyList[1] * pow(r.ptr<double>(iy, ix)[0], 5) +
                                                        vignettePolyList[2] * pow(r.ptr<double>(iy, ix)[0], 4) +
                                                        vignettePolyList[3] * pow(r.ptr<double>(iy, ix)[0], 3) +
                                                        vignettePolyList[4] * pow(r.ptr<double>(iy, ix)[0], 2) +
                                                        vignettePolyList[5] *     r.ptr<double>(iy, ix)[0] +
                                                        vignettePolyList[6]);
            }
        }
        ros::Time time_final = ros::Time::now();
        std::cout << "ImgReflectance&Lens_Corrections ----- TIME STAMP:  " << time_final - time_initial << std::endl;*/


//        cout   << r.at<double>(0, 0) << endl
//               << r.at<double>(10, 10)<<endl
//               << r.at<double>(20, 20)<<endl
//               << r.at<double>(30, 30)<<endl
//               << r.at<double>(959, 1279)<<endl;
    }


    void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv,
                  cv::Mat1i &X, cv::Mat1i &Y) {
        cv::repeat(xgv.reshape(1, 1).t(), 1, ygv.total(), X);
        cv::repeat(ygv.reshape(1, 1), xgv.total(), 1, Y);
    }

    void meshgridTest(const cv::Range &xgv, const cv::Range &ygv, cv::Mat1i &X, cv::Mat1i &Y) {
        std::vector<int> t_x, t_y;
        for (int i = xgv.start; i < xgv.end; i++) t_x.push_back(i);
        for (int i = ygv.start; i < ygv.end; i++) t_y.push_back(i);

        meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
    }


    double dist(CvPoint a, CvPoint b) {
        return sqrt(pow((double) (a.x - b.x), 2) + pow((double) (a.y - b.y), 2));
    }

// Helper function that computes the longest distance from the edge to the center point.
    double getMaxDisFromCorners(const cv::Size &imgSize, const cv::Point &center) {
        // given a rect and a line
        // get which corner of rect is farthest from the line

        std::vector<cv::Point> corners(4);
        corners[0] = cv::Point(0, 0);
        corners[1] = cv::Point(imgSize.width, 0);
        corners[2] = cv::Point(0, imgSize.height);
        corners[3] = cv::Point(imgSize.width, imgSize.height);

        double maxDis = 0;
        for (int i = 0; i < 4; ++i) {
            double dis = dist(corners[i], center);
            if (maxDis < dis)
                maxDis = dis;
        }

        return maxDis;
    }

// Helper function that creates a gradient image.
// firstPt, radius and power, are variables that control the artistic effect of the filter.
    void generateGradient(cv::Mat &mask) {
        cv::Point firstPt = cv::Point(617.117, 474.279);
        double radius = 1.0;
        double power = 0.8;


        double maxImageRad = radius * getMaxDisFromCorners(mask.size(), firstPt);


        mask.setTo(cv::Scalar(1));
        for (int i = 0; i < mask.rows; i++) {
            for (int j = 0; j < mask.cols; j++) {
                double temp = dist(firstPt, cv::Point(j, i)) / maxImageRad;
                temp = temp * power;
                double temp_s = pow(cos(temp), 4);
                mask.at<double>(i, j) = temp_s;

            }


        }

    }


    cv::Point panelHomography(cv::Mat img_scene, int &cropped_heigth, int &cropped_width, RotatedRect &rect) {
        //null_mat = cv::Mat(1, 1, CV_64F, double(0));
        cv::Point a;
        a.x = -1;
        a.y = -1;
        std::stringstream ss1;
        ss1 << ros::package::getPath("rededge_conn") << "/objects/000/QrCodePanel/QRPanel.png";
        Mat img_object = cv::imread(ss1.str(), CV_LOAD_IMAGE_GRAYSCALE);

        /*std::stringstream ss2;
        ss2 << ros::package::getPath("rededge_conn") << "/objects/000/ACalibPanels/PanelVinha/IMG_0075_1.tif";
        Mat img_scene = cv::imread(ss2.str(), CV_LOAD_IMAGE_GRAYSCALE);*/

        if (!img_object.data || !img_scene.data) {
            std::cout << " --(!) Error reading images " << std::endl;
            return a;
        }
        //-- Step 1: Detect the keypoints and extract descriptors using SURF
        int minHessian = 600;
        Ptr<SURF> detector = SURF::create(minHessian);
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;
        detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
        detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);
        //-- Step 2: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector<DMatch> matches;
        matcher.match(descriptors_object, descriptors_scene, matches);
        double max_dist = 0;
        double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for (int i = 0; i < descriptors_object.rows; i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        printf("-- Max dist : %f \n", max_dist);
        printf("-- Min dist : %f \n", min_dist);
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector<DMatch> good_matches;
        for (int i = 0; i < descriptors_object.rows; i++) {
            if (matches[i].distance < 9 * min_dist) { good_matches.push_back(matches[i]); }
        }
        Mat img_matches;
        drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for (size_t i = 0; i < good_matches.size(); i++) {
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
            scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
        }
        Mat H = findHomography(obj, scene, RANSAC);
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0, 0);
        obj_corners[1] = cvPoint(img_object.cols, 0);
        obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
        obj_corners[3] = cvPoint(0, img_object.rows);
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform(obj_corners, scene_corners, H);
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0),
             scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0),
             scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0),
             scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0),
             scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);

        cv::Point2f CornerIE;
        cv::Point2f CornerID;
        cv::Point2f CornerSE;
        cv::Point2f CornerSD;

        std::vector<float> pontos;
        pontos.push_back(scene_corners[0].x + scene_corners[0].y);
        pontos.push_back(scene_corners[1].x + scene_corners[1].y);
        pontos.push_back(scene_corners[2].x + scene_corners[2].y);
        pontos.push_back(scene_corners[3].x + scene_corners[3].y);

        auto max_pontos = std::max_element(std::begin(pontos), std::end(pontos));
        auto min_pontos = std::min_element(std::begin(pontos), std::end(pontos));
        int max_pontos_index = std::distance(pontos.begin(), max_pontos);
        int min_pontos_index = std::distance(pontos.begin(), min_pontos);

        CornerIE = scene_corners[min_pontos_index];
        CornerSD = scene_corners[max_pontos_index];

        cout << "POINT: " << scene_corners[0] << endl;
        cout << "POINT: " << scene_corners[1] << endl;
        cout << "POINT: " << scene_corners[2] << endl;
        cout << "POINT: " << scene_corners[3] << endl;
        cout << "CornerIE: " << CornerIE << endl;
        cout << "CornerSD: " << CornerSD << endl;

        //-- Show detected matches
        /*imshow("Good Matches & Object detection", img_matches);
        waitKey(1);*/
        cv::Mat out_warp;
        cropped_width = (int) abs(scene_corners[0].x - scene_corners[2].x);
        cropped_heigth = (int) abs(scene_corners[0].y - scene_corners[2].y);
        out_warp = img_scene(
                Rect((int) scene_corners[0].x, (int) scene_corners[0].y, cropped_width, cropped_heigth));
        /*imshow("OUTWarp", out_warp);
        waitKey(1);*/
        /// Approximate contours to polygons + get bounding rects
        /*vector<vector<Point2f>> contours;
        contours.push_back(scene_corners);
        vector<vector<Point> > contours_poly(contours.size());

        double maxArea = 0.0;
        for (int i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
                boundRect = boundingRect(Mat(contours_poly[i]));
            }
        }

        out_warp = img_scene(boundRect);
        imshow("cropImage", out_warp);
        waitKey(1);*/

        rect = minAreaRect(scene_corners);
        Rect boundRect = rect.boundingRect();
        int area =  boundRect.area();
        std::cout << " Area: " << area << std::endl;
        float angle = rect.angle;
        Size2f rect_size = rect.size;
        if (rect.angle < -45.) {
            angle += 90.0;
            swap(rect_size.width, rect_size.height);
        }
        Mat Mrect, rotated, cropped;
        Mrect = getRotationMatrix2D(rect.center, angle, 1.0);
        warpAffine(img_scene, rotated, Mrect, img_scene.size(), INTER_CUBIC);

        getRectSubPix(rotated, rect_size, rect.center, out_warp);
        /*imshow("cropImage", out_warp);
        cvWaitKey(1);*/


        //int ix, iy;
        Mat bin_img(out_warp.rows, out_warp.cols, CV_8UC1);
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Scalar tempVal = mean(out_warp);
        cout << "***     MEAN VAL:  " << tempVal.val[0] << endl;
        minMaxLoc(out_warp, &minVal, &maxVal, &minLoc, &maxLoc);
        for (iy = 0; iy < out_warp.rows; ++iy) {
            for (ix = 0; ix < out_warp.cols; ++ix) {
                if (out_warp.ptr<uchar>(iy, ix)[0] >= tempVal.val[0] - tempVal.val[0] * 0.2)
                    bin_img.ptr<uchar>(iy, ix)[0] = 255;
                else
                    bin_img.ptr<uchar>(iy, ix)[0] = 0;
            }
        }

        Mat frame_hist(bin_img.cols, 1, CV_8UC1);

        frame_hist = bin_img.clone();
        /*cv::imshow("CANNY 1", frame_hist);
        cvWaitKey(1);*/
        int histSize = 256;
        int hist_w = frame_hist.cols;
        int hist_h = frame_hist.rows;
        std::vector<int> hist_vertical(hist_h);
        std::vector<int> hist_horizontal(hist_w);
        //hist_vertical.resize(hist_h);
        //hist_horizontal.resize(hist_w);
        //hist_vertical.clear();
        //hist_horizontal.clear();


        /*for (iy = 0; iy < frame_hist.rows; iy++) {
            hist_vertical.push_back(0);
        }
        for (ix = 0; ix < frame_hist.cols; ix++) {
            hist_horizontal.push_back(0);
        }*/

        Mat histVert(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
        Mat histHoriz(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
         for (iy = 0; iy < frame_hist.rows; iy++) {
            for (ix = 0; ix < frame_hist.cols; ix++) {
                if ((int) frame_hist.ptr<uchar>(iy, ix)[0] == 255) {
                    hist_vertical[iy]++;
                    hist_horizontal[ix]++;
                }
                line(histHoriz, Point(ix, hist_h), Point(ix, hist_h - hist_horizontal[ix]), Scalar(0, 0, 255), 2, 8,
                     0);
            }
            line(histVert, Point(0, iy), Point(hist_vertical[iy], iy), Scalar(255, 0, 0), 2, 8, 0);
        }

        int i;
        std::vector<int>::iterator maxHorizEl = std::max_element(std::begin(hist_horizontal),
                                                                 std::end(hist_horizontal));
        int horiz_max_val = hist_horizontal[std::distance(std::begin(hist_horizontal), maxHorizEl)];
        double thresh = 0.04 * horiz_max_val;
        int first_Hproj = 0;
        int second_Hproj = 0;
        int count = 0;
        cout << "***Max Horizontal elem: ***   " << horiz_max_val << endl;
        cout << "***MinThresh: ***   " << horiz_max_val - thresh << endl;
        cout << "***MAxThresh: ***   " << horiz_max_val + thresh << endl;
        for (i = 0; i < hist_w; i++) {
            if (hist_horizontal[i] >= hist_h * 0.7) {
                horiz_max_val = hist_horizontal[i];
                thresh = 0.04 * horiz_max_val;
            }
            if (hist_horizontal[i] < horiz_max_val + thresh && hist_horizontal[i] > horiz_max_val - thresh &&
                first_Hproj == 0) {
                first_Hproj = i;
                count++;
            } else if (hist_horizontal[i] < horiz_max_val + thresh && hist_horizontal[i] > horiz_max_val - thresh &&
                       first_Hproj != 0) {
                second_Hproj = i;
                count++;
            } else {
                //Se ocupar mais de 28% da imagem -> Painel
                if (count >= 0.28 * hist_w) {
                    i = hist_w;
                    continue;
                }
                first_Hproj = 0;
                second_Hproj = 0;
                count = 0;
            }
        }
        cout << "***HORIZONTAL 1: *** FIRST ELEM:   " << first_Hproj << endl;
        cout << "***HORIZONTAL 1: *** SECOND ELEM:   " << second_Hproj << endl;

        int first_Vproj = 0;
        int second_Vproj = 0;
        int vert_max_val = 0;
        cout << "**** SIZE HIST VERTICAL:   " << hist_vertical.size() << endl;
        cout << "**** SIZE HIST HORIZONTAL:   " << hist_horizontal.size() << endl;
        cout << "HST_WIDTH: " << hist_w << "   65% de hist:  " << hist_w * 0.65 << endl;
        for (i = 0; i < hist_vertical.size(); i++) {
            if (hist_vertical[i] >= hist_w * 0.65) {
                first_Vproj = i;
                i = hist_h;
            }
        }
        vert_max_val = 0;
        for (i = hist_vertical.size() - 1; i >= 0; i--) {
            cout << "**** VALUE i:   " << i << "    HIST VALUE: " << hist_vertical[i] << endl;
            cout << "**** SIZE HIST VERTICAL:   " << hist_vertical.size() << endl;
            if (hist_vertical[i] >= hist_w * 0.65) {
                second_Vproj = i;
                i = -1;
            }
        }
        cout << "***VERTICAL 1: *** FIRST ELEM:   " << first_Vproj << endl;
        cout << "***VERTICAL 1: *** SECOND ELEM:   " << second_Vproj << endl;

        /*imshow("Horizontal Histogram", histHoriz);
        cvWaitKey(1);

        imshow("Vertical Histogram", histVert);
        cvWaitKey(1);*/

        UpLeft_clicked.x = first_Hproj;
        UpLeft_clicked.y = first_Vproj;
        cropped_heigth = second_Vproj - first_Vproj;
        cropped_width = second_Hproj - first_Hproj;
        cv::Mat panelRegion;
        panelRegion = out_warp(
                Rect(UpLeft_clicked.x, UpLeft_clicked.y, cropped_width, cropped_heigth));
        //imshow("MarkedImage", panelRegion);
        //waitKey(1);
        //UpLeft_clicked.x = (int) CornerIE.x + first_Hproj;
        //UpLeft_clicked.y = (int) CornerIE.y + first_Vproj;

        return UpLeft_clicked;
    }

    int TestpanelHomography() {

        cv::Point a;
        a.x = 0;
        a.y = 0;
        std::stringstream ss1;
        ss1 << ros::package::getPath("rededge_conn") << "/objects/000/QrCodePanel/QRPanel.png";
        Mat img_object = cv::imread(ss1.str(), CV_LOAD_IMAGE_GRAYSCALE);

        std::stringstream ss2;
        ss2 << ros::package::getPath("rededge_conn") << "/objects/000/ACalibPanels/IMG_0000_A.tif";
        Mat img_scene = cv::imread(ss2.str(), CV_LOAD_IMAGE_GRAYSCALE);

        imshow("Original Image", img_scene);
        cvWaitKey(1);
        /*std::stringstream ss2;
        ss2 << ros::package::getPath("rededge_conn") << "/objects/000/ACalibPanels/PanelVinha/IMG_0075_1.tif";
        Mat img_scene = cv::imread(ss2.str(), CV_LOAD_IMAGE_GRAYSCALE);*/

        if (!img_object.data || !img_scene.data) {
            std::cout << " --(!) Error reading images " << std::endl;
            return -1;
        }
        //-- Step 1: Detect the keypoints and extract descriptors using SURF
        int minHessian = 600;
        Ptr<SURF> detector = SURF::create(minHessian);
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;
        detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
        detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);
        //-- Step 2: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector<DMatch> matches;
        matcher.match(descriptors_object, descriptors_scene, matches);
        double max_dist = 0;
        double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for (int i = 0; i < descriptors_object.rows; i++) {
            double dist = matches[i].distance;
            if (dist < min_dist) min_dist = dist;
            if (dist > max_dist) max_dist = dist;
        }
        printf("-- Max dist : %f \n", max_dist);
        printf("-- Min dist : %f \n", min_dist);
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector<DMatch> good_matches;
        for (int i = 0; i < descriptors_object.rows; i++) {
            if (matches[i].distance < 9 * min_dist) { good_matches.push_back(matches[i]); }
        }
        Mat img_matches;
        drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for (size_t i = 0; i < good_matches.size(); i++) {
            //-- Get the keypoints from the good matches
            obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
            scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
        }
        Mat H = findHomography(obj, scene, RANSAC);
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0, 0);
        obj_corners[1] = cvPoint(img_object.cols, 0);
        obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
        obj_corners[3] = cvPoint(0, img_object.rows);
        std::vector<Point2f> scene_corners(4);
        perspectiveTransform(obj_corners, scene_corners, H);
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0),
             scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0),
             scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0),
             scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
        line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0),
             scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);

        cv::Point2f CornerIE;
        cv::Point2f CornerID;
        cv::Point2f CornerSE;
        cv::Point2f CornerSD;

        std::vector<float> pontos;
        pontos.push_back(scene_corners[0].x + scene_corners[0].y);
        pontos.push_back(scene_corners[1].x + scene_corners[1].y);
        pontos.push_back(scene_corners[2].x + scene_corners[2].y);
        pontos.push_back(scene_corners[3].x + scene_corners[3].y);

        auto max_pontos = std::max_element(std::begin(pontos), std::end(pontos));
        auto min_pontos = std::min_element(std::begin(pontos), std::end(pontos));
        int max_pontos_index = std::distance(pontos.begin(), max_pontos);
        int min_pontos_index = std::distance(pontos.begin(), min_pontos);

        CornerIE = scene_corners[min_pontos_index];
        CornerSD = scene_corners[max_pontos_index];

        cout << "POINT: " << scene_corners[0] << endl;
        cout << "POINT: " << scene_corners[1] << endl;
        cout << "POINT: " << scene_corners[2] << endl;
        cout << "POINT: " << scene_corners[3] << endl;
        cout << "CornerIE: " << CornerIE << endl;
        cout << "CornerSD: " << CornerSD << endl;

        //-- Show detected matches
        imshow("Good Matches & Object detection", img_matches);
        waitKey(1);
        cv::Mat out_warp;
        int cropped_width = (int) abs(scene_corners[0].x - scene_corners[2].x);
        int cropped_heigth = (int) abs(scene_corners[0].y - scene_corners[2].y);
        out_warp = img_scene(
                Rect((int) scene_corners[0].x, (int) scene_corners[0].y, cropped_width, cropped_heigth));
        /*imshow("OUTWarp", out_warp);
        waitKey(1);*/
        /// Approximate contours to polygons + get bounding rects
        vector<vector<Point2f>> contours;
        contours.push_back(scene_corners);
        vector<vector<Point> > contours_poly(contours.size());
        /*cv::Rect boundRect;
        double maxArea = 0.0;
        for (int i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
                boundRect = boundingRect(Mat(contours_poly[i]));
            }
        }

        out_warp = img_scene(boundRect);
        imshow("cropImage", out_warp);
        waitKey(0);*/

        RotatedRect rect = minAreaRect(scene_corners);
        Rect bound = rect.boundingRect();
        int area =  bound.area();
        std::cout << " Area: " << area << std::endl;

        float angle = rect.angle;
        Size2f rect_size = rect.size;
        if (rect.angle < -45.) {
            angle += 90.0;
            swap(rect_size.width, rect_size.height);
        }
        Mat Mrect, rotated, cropped;
        Mrect = getRotationMatrix2D(rect.center, angle, 1.0);
        warpAffine(img_scene, rotated, Mrect, img_scene.size(), INTER_CUBIC);
        getRectSubPix(rotated, rect_size, rect.center, out_warp);
        //out_warp = img_scene(boundRect);
        /*imshow("cropImage", out_warp);
        waitKey(1);*/

        //int ix, iy;
        Mat bin_img(out_warp.rows, out_warp.cols, CV_8UC1);
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Scalar tempVal = mean(out_warp);
        cout << "***     MEAN VAL:  " << tempVal.val[0] << endl;
        minMaxLoc(out_warp, &minVal, &maxVal, &minLoc, &maxLoc);
        for (iy = 0; iy < out_warp.rows; ++iy) {
            for (ix = 0; ix < out_warp.cols; ++ix) {
                if (out_warp.ptr<uchar>(iy, ix)[0] >= tempVal.val[0] - tempVal.val[0] * 0.2)
                    bin_img.ptr<uchar>(iy, ix)[0] = 255;
                else
                    bin_img.ptr<uchar>(iy, ix)[0] = 0;
            }
        }

        Mat frame_hist(bin_img.cols, 1, CV_8UC1);

        frame_hist = bin_img.clone();
        cv::imshow("CANNY 1", frame_hist);
        cvWaitKey(1);
        int histSize = 256;
        int hist_w = frame_hist.cols;
        int hist_h = frame_hist.rows;
        std::vector<int> hist_vertical(hist_h);
        std::vector<int> hist_horizontal(hist_w);
        //hist_vertical.resize(hist_h);
        //hist_horizontal.resize(hist_w);
        //hist_vertical.clear();
        //hist_horizontal.clear();


        /*for (iy = 0; iy < frame_hist.rows; iy++) {
            hist_vertical.push_back(0);
        }
        for (ix = 0; ix < frame_hist.cols; ix++) {
            hist_horizontal.push_back(0);
        }*/

        Mat histVert(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
        Mat histHoriz(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
        for (iy = 0; iy < frame_hist.rows; iy++) {
            for (ix = 0; ix < frame_hist.cols; ix++) {
                if ((int) frame_hist.ptr<uchar>(iy, ix)[0] == 255) {
                    hist_vertical[iy]++;
                    hist_horizontal[ix]++;
                }
                line(histHoriz, Point(ix, hist_h), Point(ix, hist_h - hist_horizontal[ix]), Scalar(0, 0, 255), 2, 8,
                     0);
            }
            line(histVert, Point(0, iy), Point(hist_vertical[iy], iy), Scalar(255, 0, 0), 2, 8, 0);
        }

        int i;
        std::vector<int>::iterator maxHorizEl = std::max_element(std::begin(hist_horizontal),
                                                                 std::end(hist_horizontal));
        int horiz_max_val = hist_horizontal[std::distance(std::begin(hist_horizontal), maxHorizEl)];
        double thresh = 0.04 * horiz_max_val;
        int first_Hproj = 0;
        int second_Hproj = 0;
        int count = 0;
        cout << "***Max Horizontal elem: ***   " << horiz_max_val << endl;
        cout << "***MinThresh: ***   " << horiz_max_val - thresh << endl;
        cout << "***MAxThresh: ***   " << horiz_max_val + thresh << endl;
        for (i = 0; i < hist_w; i++) {
            if (hist_horizontal[i] >= hist_h * 0.7) {
                horiz_max_val = hist_horizontal[i];
                thresh = 0.04 * horiz_max_val;
            }
            if (hist_horizontal[i] < horiz_max_val + thresh && hist_horizontal[i] > horiz_max_val - thresh &&
                first_Hproj == 0) {
                first_Hproj = i;
                count++;
            } else if (hist_horizontal[i] < horiz_max_val + thresh && hist_horizontal[i] > horiz_max_val - thresh &&
                       first_Hproj != 0) {
                second_Hproj = i;
                count++;
            } else {
                //Se ocupar mais de 28% da imagem -> Painel
                if (count >= 0.28 * hist_w) {
                    i = hist_w;
                    continue;
                }
                first_Hproj = 0;
                second_Hproj = 0;
                count = 0;
            }
        }
        cout << "***HORIZONTAL 1: *** FIRST ELEM:   " << first_Hproj << endl;
        cout << "***HORIZONTAL 1: *** SECOND ELEM:   " << second_Hproj << endl;

        int first_Vproj = 0;
        int second_Vproj = 0;
        int vert_max_val = 0;
        cout << "**** SIZE HIST VERTICAL:   " << hist_vertical.size() << endl;
        cout << "**** SIZE HIST HORIZONTAL:   " << hist_horizontal.size() << endl;
        cout << "HST_WIDTH: " << hist_w << "   65% de hist:  " << hist_w * 0.65 << endl;
        for (i = 0; i < hist_vertical.size(); i++) {
            if (hist_vertical[i] >= hist_w * 0.65) {
                first_Vproj = i;
                i = hist_h;
            }
        }
        vert_max_val = 0;
        for (i = hist_vertical.size() - 1; i >= 0; i--) {
            cout << "**** VALUE i:   " << i << "    HIST VALUE: " << hist_vertical[i] << endl;
            cout << "**** SIZE HIST VERTICAL:   " << hist_vertical.size() << endl;
            if (hist_vertical[i] >= hist_w * 0.65) {
                second_Vproj = i;
                i = -1;
            }
        }
        cout << "***VERTICAL 1: *** FIRST ELEM:   " << first_Vproj << endl;
        cout << "***VERTICAL 1: *** SECOND ELEM:   " << second_Vproj << endl;

        imshow("Horizontal Histogram", histHoriz);
        cvWaitKey(0);

        imshow("Vertical Histogram", histVert);
        cvWaitKey(0);

        UpLeft_clicked.x = first_Hproj;
        UpLeft_clicked.y = first_Vproj;
        cropped_heigth = second_Vproj - first_Vproj;
        cropped_width = second_Hproj - first_Hproj;
        cv::Mat panelRegion;
        panelRegion = out_warp(
                Rect(UpLeft_clicked.x, UpLeft_clicked.y, cropped_width, cropped_heigth));
       /* imshow("MarkedImage", panelRegion);
        waitKey(1);*/
        //UpLeft_clicked.x = (int) CornerIE.x + first_Hproj;
        //UpLeft_clicked.y = (int) CornerIE.y + first_Vproj;

        return 0;
    }
}
