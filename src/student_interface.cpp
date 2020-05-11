#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "dubins.hpp"
#include "hsv_panel.hpp"

#include <experimental/filesystem>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <limits>
#include <algorithm>

#define AUTO_CORNER_DETECTION false

// -------------------------------- DEBUG FLAGS --------------------------------
// - Configuration Debug flags - //
// #define DEBUG_COLOR_RANGE // Prints info about HSV ranges used for colors
// #define DEBUG_COLOR_CONFIG

// - Calibration Debug flags - //
// #define DEBUG_ESTRINISIC_CALIB

// - Image Analysis Debug flags - //
// #define DEBUG_FINDOBSTACLES
// #define DEBUG_FINDGATE
// #define DEBUG_FINDVICTIMS
// #define DEBUG_FINDROBOT

// - Planning Debug flags - //
// #define DEBUG_PLANPATH            // generic info about the whole planner
// #define DEBUG_RRT                 // inner planning algorithm
// #define DEBUG_PATH_SMOOTHING      // path smoothing pipeline
#define DEBUG_DRAWCURVE             // dubins path plotting
// #define DEBUG_COLLISION           // plot for collision detection

using namespace std;

// --------------------------------- CONSTANTS ---------------------------------
const string COLOR_CONFIG_FILE = "/color_parameters.config";
const int pythonUpscale = 1000; // scale factor used to convert parameters to a
                                // int represetation for the planning library
const double debugImagesScale = 512.82; // arbitrary scale factor used for
                                        // displaying debug images

namespace student {

struct Color_config {
    tuple<int,int,int> victims_lowbound;
    tuple<int,int,int> victims_highbound;
    tuple<int,int,int> robot_lowbound;
    tuple<int,int,int> robot_highbound;
    tuple<int,int,int> obstacle_lowbound1;
    tuple<int,int,int> obstacle_highbound1;
    tuple<int,int,int> obstacle_lowbound2;
    tuple<int,int,int> obstacle_highbound2;
};

#ifdef DEBUG_DRAWCURVE
//-------------------DRAWING DUBINS CURVES-------------------
cv::Mat dcImg = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255,255,255));
#endif

void loadImage(cv::Mat& img_out, const string& config_folder) {
    static bool initialized = false;
    static vector<cv::String> img_list; // list of images to load
    static size_t idx = 0;  // idx of the current img
    static size_t function_call_counter = 0;  // idx of the current img
    const static size_t freeze_img_n_step = 30; // hold the current image for n iteration
    static cv::Mat current_img; // store the image for a period, avoid to load it from file every time

    if (!initialized) {
        const bool recursive = false;
        // Load the list of jpg image contained in the config_folder/img_to_load/
        cv::glob(config_folder + "/img_to_load/*.jpg", img_list, recursive);

        if (img_list.size() > 0) {
            initialized = true;
            idx = 0;
            current_img = cv::imread(img_list[idx]);
            function_call_counter = 0;
        } else {
            initialized = false;
        }
    }

    if (!initialized) {
        throw logic_error("Load Image can not find any jpg image in: " +  config_folder + "/img_to_load/");
        return;
    }

    img_out = current_img;
    function_call_counter++;

    // If the function is called more than N times load increment image idx
    if (function_call_counter > freeze_img_n_step) {
        function_call_counter = 0;
        idx = (idx + 1)%img_list.size();
        current_img = cv::imread(img_list[idx]);
    }
}

void genericImageListener(const cv::Mat& img_in, string topic,
                          const string& config_folder) {

        static int imageCounter= 0;

        cv::imshow("current picture", img_in);
        char c = cv::waitKey(30);
        if (c == 's') {
        /* Create folder for images*/
        string foldername = config_folder;
        foldername += "/image";      //This is the folder used in the topic string
        string command = "mkdir -p " + foldername;  //-p creates only if non exists
        system(command.c_str());  //use bash command

        /* Save current image */
        string filename = config_folder;
        filename += topic;
        filename += "_";
        filename += to_string(imageCounter++);
        filename += ".jpg";
        cv::imwrite(filename, img_in);
        printf("Saved is '%s'\n", filename.c_str());
    }
}

bool autodetect_corners(const cv::Mat& img_in, vector<cv::Point2f>& corners) {
    // convert to grayscale (you could load as grayscale instead)
    // cv::Mat gray;
    // cv::cvtColor(img_in,gray, CV_BGR2GRAY);
    //
    // // compute mask (you could use a simple threshold if the image is always as good as the one you provided)
    // cv::Mat mask;
    // cv::threshold(gray, mask, 10, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    //
    //
    // // find contours (if always so easy to segment as your image, you could just add the black/rect pixels to a vector)
    // vector<vector<cv::Point>> contours;
    // vector<cv::Vec4i> hierarchy;
    // cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //
    // /// Draw contours and find biggest contour (if there are other contours in the image, we assume the biggest one is the desired rect)
    // // drawing here is only for demonstration!
    // int biggestContourIdx = -1;
    // float biggestContourArea = 0;
    // // cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC3); REMOVE
    //
    // for (int i = 0; i< contours.size(); i++)
    // {
    //     // cv::Scalar color = cv::Scalar(0, 100, 0); REMOVE
    //     //drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point()); REMOVE
    //
    //     float ctArea= cv::contourArea(contours[i]);
    //     if (ctArea > biggestContourArea)
    //     {
    //         biggestContourArea = ctArea;
    //         biggestContourIdx = i;
    //     }
    // }
    //
    // // if no contour found
    // if (biggestContourIdx < 0)
    // {
    //     cout << "no contour found" << endl;
    //     return false;
    // }
    //
    // // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
    // cv::RotatedRect boundingBox = cv::minAreaRect(contours[biggestContourIdx]);
    // // one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines
    //
    // boundingBox.points(corners);
    return true;
}

void onMouse(int evt, int x, int y, int flags, void* param) {
    if (evt == CV_EVENT_LBUTTONDOWN) {
        vector<cv::Point>* ptPtr = (vector<cv::Point>*)param;
        ptPtr->push_back(cv::Point(x,y));
    }
}

void manualselect_corners(const cv::Mat& img_in, vector<cv::Point2f>& corners) {
    vector<cv::Point> points;
    string windowname = "Select corners, counterclockwise, start from red";
    cv::namedWindow(windowname);
    cv::setMouseCallback(windowname, onMouse, (void*)&points);

    while (points.size() < 4) {
        cv::imshow(windowname, img_in);

        for (int i=0; i < points.size(); i++) {
            cv::circle(img_in, points[i], 20, cv::Scalar(240,0,0),CV_FILLED);
        }
        cv::waitKey(1);
    }
    cv::destroyWindow(windowname);

    for (int i=0; i < 4; i++) {
        corners.push_back(points[i]);
    }
}

Color_config read_colors(const string& config_folder) {
    Color_config color_config;

    string file_path = config_folder;
    file_path += "/";
    file_path += COLOR_CONFIG_FILE;

    if (experimental::filesystem::exists(file_path)) {
        #ifdef DEBUG_COLOR_CONFIG
            printf("Reading color configuration\n");
        #endif

        // Load configuration from file
        ifstream input(file_path);
        if (!input.is_open()) {
            throw runtime_error("Cannot read file: " + file_path);
        }
        while (!input.eof()) {
            string name;
            int v1,v2,v3;
            if (!(input >> name >> v1 >> v2 >> v3)) {
                if (input.eof())
                    break;
                else
                    throw runtime_error("Malformed file: " + file_path);
            }

            #ifdef DEBUG_COLOR_CONFIG
                printf("---> Reading %s : %d,%d,%d\n",name.c_str(),v1,v2,v3);
            #endif

            if (name.compare("victims_lowbound")==0)
                color_config.victims_lowbound = make_tuple(v1,v2,v3);
            else if (name.compare("victims_highbound")==0)
                color_config.victims_highbound= make_tuple(v1,v2,v3);
            else if (name.compare("robot_lowbound")==0)
                color_config.robot_lowbound= make_tuple(v1,v2,v3);
            else if (name.compare("robot_highbound")==0)
                color_config.robot_highbound= make_tuple(v1,v2,v3);
            else if (name.compare("obstacle_lowbound1")==0)
                color_config.obstacle_lowbound1= make_tuple(v1,v2,v3);
            else if (name.compare("obstacle_highbound1")==0)
                color_config.obstacle_highbound1= make_tuple(v1,v2,v3);
            else if (name.compare("obstacle_lowbound2")==0)
                color_config.obstacle_lowbound2= make_tuple(v1,v2,v3);
            else if (name.compare("obstacle_highbound2")==0)
                color_config.obstacle_highbound2= make_tuple(v1,v2,v3);
            else
                throw runtime_error("Wrong coefficient: " + file_path);
        }
        input.close();
    }
    return color_config;
}

/*
* Open a series of panels to tune the color threshold for better detection
*/
void tune_color_parameters(const cv::Mat &image, const string& config_folder) {
    // Set destination file
    string file_path = config_folder;
    file_path += "/";
    file_path += COLOR_CONFIG_FILE;
    // Call routine in panel library
    hsvpanel::show_panel(image,file_path);
}

bool extrinsicCalib(const cv::Mat& img_in, vector<cv::Point3f> object_points,
                    const cv::Mat& camera_matrix, cv::Mat& rvec,
                    cv::Mat& tvec, const string& config_folder) {
    vector<cv::Point2f> corners;

    if (AUTO_CORNER_DETECTION)
        autodetect_corners(img_in,corners);
    else {
        ///Try to read calibration file
        string file_path = config_folder + "/extrinsicCalib.csv";

        if (!experimental::filesystem::exists(file_path)) {
            // File does not exist
            manualselect_corners(img_in,corners);
            // Save the file
            experimental::filesystem::create_directories(config_folder);
            ofstream output(file_path);
            if (!output.is_open()) {
                throw runtime_error("Cannot write file: " + file_path);
            }
            for (const auto pt: corners) {
                output << pt.x << " " << pt.y << endl;
            }
            output.close();
        } else {
            // Load configuration from file
            ifstream input(file_path);
            if (!input.is_open()) {
                throw runtime_error("Cannot read file: " + file_path);
            }
            while (!input.eof()) {
                double x, y;
                if (!(input >> x >> y)) {
                    if (input.eof())
                        break;
                    else
                      throw runtime_error("Malformed file: " + file_path);
                }
                corners.emplace_back(x, y);
            }
            input.close();
        }
    }

    #ifdef DEBUG_ESTRINISIC_CALIB
        cv::line(img_in, corners[0], corners[1], cv::Scalar(0,0,255));
        cv::line(img_in, corners[1], corners[2], cv::Scalar(0,0,255));
        cv::line(img_in, corners[2], corners[3], cv::Scalar(0,0,255));
        cv::line(img_in, corners[3], corners[0], cv::Scalar(0,0,255));

        cv::circle(img_in, corners[0], 20, cv::Scalar(50,50,50),4);
        cv::circle(img_in, corners[1], 20, cv::Scalar(50,50,50),4);
        cv::circle(img_in, corners[2], 20, cv::Scalar(50,50,50),4);
        cv::circle(img_in, corners[3], 20, cv::Scalar(50,50,50),4);
        // display
        cv::imshow("Selected border points", img_in);
        cv::waitKey(0);
    #endif

    cv::Mat nullmat;
    cv::solvePnP(object_points,corners, camera_matrix, nullmat, rvec, tvec);

    // Call the routine with gui to tune the color values
    //tune_color_parameters(img_in, config_folder);
}

void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out,
                    const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs,
                    const string& config_folder) {

    cv::undistort(img_in,img_out,cam_matrix,dist_coeffs);
}

void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec,
                        const cv::Mat& tvec,
                        const vector<cv::Point3f>& object_points_plane,
                        const vector<cv::Point2f>& dest_image_points_plane,
                        cv::Mat& plane_transf, const string& config_folder) {

    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
}

void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf,
            const string& config_folder) {

    cv::warpPerspective(img_in, img_out, transf, img_in.size());
}

void findObstacles(const cv::Mat& hsv_img, const double scale,
                   vector<Polygon>& obstacle_list,
                   const Color_config& color_config) {

    /* Red color requires 2 ranges*/
    cv::Mat lower_red_hue_range; // the lower range for red hue
    cv::Mat upper_red_hue_range; // the higher range for red hue

    auto t = color_config.obstacle_lowbound1;
    int lowH1 = get<0>(t);
    int lowS1 = get<1>(t);
    int lowV1 = get<2>(t);
    t = color_config.obstacle_highbound1;
    int highH1 = get<0>(t);
    int highS1 = get<1>(t);
    int highV1 = get<2>(t);
    #ifdef DEBUG_COLOR_RANGE
        printf("Using RED bound 1 (%d,%d,%d)-(%d,%d,%d)\n", lowH1, lowS1, lowV1, highH1, highS1, highV1);
    #endif

    t = color_config.obstacle_lowbound2;
    int lowH2 = get<0>(t);
    int lowS2 = get<1>(t);
    int lowV2 = get<2>(t);
    t = color_config.obstacle_highbound2;
    int highH2 = get<0>(t);
    int highS2 = get<1>(t);
    int highV2 = get<2>(t);
    #ifdef DEBUG_COLOR_RANGE
        printf("Using RED bound 2 (%d,%d,%d)-(%d,%d,%d)\n", lowH2, lowS2,  lowV2, highH2, highS2, highV2);
    #endif
    cv::inRange(hsv_img, cv::Scalar(lowH1, lowS1, lowV1), cv::Scalar(highH1, highS1, highV1), lower_red_hue_range);
    cv::inRange(hsv_img, cv::Scalar(lowH2, lowS2, lowV2), cv::Scalar(highH2, highS2, highV2), upper_red_hue_range);

    // Now we can combine the 2 masks
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    //This can reduce false positives

    // compute robot dimension from barycenter for obstacle dilation
    // distance between robot triangle front vertex and barycenter is triangle height/3*2
    // from documentation, triangle height is 16 cm
    float robot_dim = ceil(0.1117*scale);

    cout << "robot dim: " << robot_dim << endl;

    // dilate obstacles
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(robot_dim, robot_dim));
    cv::dilate(red_hue_image, red_hue_image, kernel);

    /*
    * Now the mask has to undergo contour detection
    */

    vector<vector<cv::Point>> contours, contours_approx;
    vector<cv::Point> approx_curve;
    cv::Mat contours_img;
    contours_img = hsv_img.clone();

    cv::findContours(red_hue_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 3, cv::LINE_AA); //REMOVE
    for (int i=0; i<contours.size(); ++i) {
        approxPolyDP(contours[i], approx_curve, 3, true);

        Polygon scaled_contour;
        for (const auto& pt: approx_curve) {
            scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
        }
        obstacle_list.push_back(scaled_contour);

        contours_approx = {approx_curve};
        cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }

    #ifdef DEBUG_FINDOBSTACLES
        imshow("obstacles", contours_img);
        cv::waitKey(0);
    #endif
}

bool findGate(const cv::Mat& hsv_img, const double scale, Polygon& gate,
             const Color_config& color_config) {

    // Find green regions
    auto t = color_config.victims_lowbound;
    int lowH = get<0>(t);
    int lowS = get<1>(t);
    int lowV = get<2>(t);
    t = color_config.victims_highbound;
    int highH = get<0>(t);
    int highS = get<1>(t);
    int highV = get<2>(t);

    #ifdef DEBUG_COLOR_RANGE
        printf("Using GREEN bound  (%d,%d,%d)-(%d,%d,%d)\n", lowH, lowS, lowV, highH, highS, highV);
    #endif

    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), green_mask);

    vector<vector<cv::Point>> contours, contours_approx;
    vector<cv::Point> approx_curve;
    cv::Mat contours_img;
    contours_img = hsv_img.clone();

    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool res = false;

    for (auto& contour : contours) {
        //const double area = cv::contourArea(contour);
        //cout << "AREA " << area << endl;
        //cout << "SIZE: " << contours.size() << endl;

        approxPolyDP(contour, approx_curve, 10, true);

        if (approx_curve.size() != 4) continue;

        contours_approx = {approx_curve};
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

        for (const auto& pt: approx_curve) {
          gate.emplace_back(pt.x/scale, pt.y/scale);
        }

        res = true;
    }

    #ifdef DEBUG_FINDGATE
        cv::imshow("findGate", contours_img);
        cv::waitKey(0);
    #endif

    return res;
}

bool findVictims(const cv::Mat& hsv_img, const double scale,
                 vector<pair<int,Polygon>>& victim_list,
                 const Color_config& color_config,
                 const string& config_folder) {

    // Find green regions
    auto t = color_config.victims_lowbound;
    int lowH = get<0>(t);
    int lowS = get<1>(t);
    int lowV = get<2>(t);
    t = color_config.victims_highbound;
    int highH = get<0>(t);
    int highS = get<1>(t);
    int highV = get<2>(t);

    #ifdef DEBUG_COLOR_RANGE
        printf("Using GREEN bound  (%d,%d,%d)-(%d,%d,%d)\n", lowH, lowS, lowV, highH, highS, highV);
    #endif

    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), green_mask);

    vector<vector<cv::Point>> contours, contours_approx;
    vector<cv::Point> approx_curve;
    cv::Mat contours_img;
    contours_img = hsv_img.clone();

    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    vector<cv::Rect> boundRect(contours.size());
    vector<cv::RotatedRect> minRect(contours.size());

    for (int i=0; i<contours.size(); ++i) {
        approxPolyDP(contours[i], approx_curve, 10, true);

        if (approx_curve.size() != 4) {  // ignore gate

            Polygon scaled_contour;
            for (const auto& pt: approx_curve) {
                scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
            }
            victim_list.push_back({i+1, scaled_contour});

            contours_approx = {approx_curve};
            drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
            boundRect[i] = boundingRect(cv::Mat(approx_curve)); // find bounding box for each green blob
            minRect[i] = minAreaRect(cv::Mat(contours[i]));
        }
    }

    #ifdef DEBUG_FINDVICTIMS
        cv::imshow("findVictims", contours_img);
        cv::waitKey(0);
    #endif
    // TEMPLATE MATCHING

    cv::Mat img;
    cv::cvtColor(hsv_img, img, cv::COLOR_HSV2BGR);

    // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
    cv::Mat green_mask_inv, filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255,255,255));
    cv::bitwise_not(green_mask, green_mask_inv);

    #ifdef DEBUG_FINDVICTIMS
        cv::imshow("Numbers", green_mask_inv);
        cv::waitKey(0);
    #endif
    cv::Mat curr_num;

    // Load digits template images
    vector<cv::Mat> templROIs;
    for (int i=0; i<=5; ++i) {
        curr_num = cv::imread(config_folder + "/../imgs/template/" + to_string(i) + ".png");
        templROIs.emplace_back(curr_num);

        for (int j = 0; j < 3; ++j) {
            cv::rotate(curr_num, curr_num, cv::ROTATE_90_CLOCKWISE);
            templROIs.emplace_back(curr_num);
        }
    }

    img.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));

    contours.clear();

    int curr_victim = 0;

    // For each green blob in the original image containing a digit
    for (int i=0; i < boundRect.size(); ++i) {
        cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit

        // FLIP HERE
        cv::flip(processROI, processROI, 0);

        if (processROI.empty()) continue;

        cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
        cv::threshold(processROI, processROI, 100, 255, 0); // threshold and binarize the image, to suppress some noise

        // Apply some additional smoothing and filtering
        cv::erode(processROI, processROI, kernel);
        cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
        cv::erode(processROI, processROI, kernel);

        // advanced ROI with rotation
        cv::cvtColor(processROI, processROI, cv::COLOR_BGR2GRAY);
        cv::bitwise_not(processROI, processROI);

        // extract minimum rectangle enclosing the number
        cv::findContours(processROI, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        minRect[i] = minAreaRect(cv::Mat(contours[0]));

        // draw min rectangle       //TODO: keep or remove? decide
        //cv::Point2f rect_points[4];
        //minRect[i].points(rect_points);
        //for (int k = 0; k < 4; ++k) {
        //   line(processROI, rect_points[k], rect_points[(k+1)%4], cv::Scalar(255,0,255), 1, 8);
        //}

        // rotate min rectangle to align with axes
        cv::Mat rotM = cv::getRotationMatrix2D(minRect[i].center, minRect[i].angle, 1.0);
        cv::warpAffine(processROI, processROI, rotM, processROI.size(), cv::INTER_CUBIC);
        cv::bitwise_not(processROI, processROI);
        cv::cvtColor(processROI, processROI, cv::COLOR_GRAY2BGR);

        // Show the actual image used for the template matching
        #ifdef DEBUG_FINDVICTIMS
            cv::imshow("ROI", processROI);
        #endif

        // Find the template digit with the best matching
        double maxScore = 0;
        int maxIdx = -1;
        for (int j = 0; j < templROIs.size(); ++j) {
            cv::Mat result;
            cv::matchTemplate(processROI, templROIs[j], result, cv::TM_CCOEFF);
            double score;
            cv::minMaxLoc(result, nullptr, &score);
            if (score > maxScore) {
                maxScore = score;
                maxIdx = floor(j/4);
            }
        }

        victim_list[curr_victim].first = maxIdx;
        curr_victim++;

        cout << "Best fitting template: " << maxIdx << endl;
        cv::waitKey(0);
    }

    return true;
}

bool processMap(const cv::Mat& img_in, const double scale,
                vector<Polygon>& obstacle_list,
                vector<pair<int,Polygon>>& victim_list,
                Polygon& gate, const string& config_folder) {

    // Convert to HSV for better color detection
    cv::Mat img_hsv;
    cv::cvtColor(img_in, img_hsv, cv::COLOR_BGR2HSV);

    Color_config color_config = read_colors(config_folder);

    findGate(img_hsv, scale, gate,color_config);
    findObstacles(img_hsv, scale, obstacle_list,color_config);
    findVictims(img_hsv, scale, victim_list, color_config, config_folder);

    return true;
}

/*!
* Finds the baricenter of a utils::Polygon
*/
void baricenter(const Polygon& polygon, double& cx, double& cy) {
    for (auto vertex: polygon) {
        cx += vertex.x;
        cy += vertex.y;
    }
    cx /= static_cast<double>(polygon.size());
    cy /=  static_cast<double>(polygon.size());
}

Point baricenter(const Polygon& polygon) {
    double cx,cy;
    baricenter(polygon,cx,cy);
    return Point(cx,cy);
}

bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle,
               double& x, double& y, double& theta,
               const string& config_folder) {
    Color_config color_config = read_colors(config_folder);

    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    // Extract blue color region
    auto t = color_config.robot_lowbound;
    int lowH = get<0>(t);
    int lowS = get<1>(t);
    int lowV = get<2>(t);
    t = color_config.robot_highbound;
    int highH = get<0>(t);
    int highS = get<1>(t);
    int highV = get<2>(t);

    #ifdef DEBUG_COLOR_RANGE
        printf("Using BLUE bound  (%d,%d,%d)-(%d,%d,%d)\n", lowH, lowS, lowV, highH, highS, highV);
    #endif
    cv::Mat blue_mask;
    cv::inRange(hsv_img, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), blue_mask);

    // remove noise with opening operation
    //cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

    // find robot contours and approximate to triangle
    vector<vector<cv::Point>> contours;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    #ifdef DEBUG_FINDROBOT
        cv::Mat contours_img;
        contours_img = img_in.clone();
    #endif

    vector<cv::Point> approx_curve;
    vector<vector<cv::Point>> contours_approx;
    bool found = false;
    for (int i=0; i<contours.size(); ++i) {
        // Approximate the i-th contours
        cv::approxPolyDP(contours[i], approx_curve, 10, true);

        // Check the number of edge of the aproximated contour
        if (approx_curve.size() != 3) continue;

        // draw approximated contours
        #ifdef DEBUG_FINDROBOT
            //cout << "Approx contour count: " << approx_curve.size() << endl;
            contours_approx = {approx_curve};
            cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,0,255), 1, cv::LINE_AA);

            cv::imshow("findRobot", contours_img);
            cv::waitKey(0);
        #endif
        found = true;
    }

    // set robot position
    if (found) {
        // emplace back every vertex on triangle (output of this function)
        for (const auto& pt: approx_curve) {
            triangle.emplace_back(pt.x/scale, pt.y/scale);
            // remember to use the scale to convert the position on the image
            // (pixels) to the position in the arena (meters)
        }

        // Find the position of the robot (baricenter of the triangle)
        double cx = 0, cy = 0;

        // Compute the triangle baricenter
        baricenter(triangle,cx,cy);

        // Find the robot orientation (i.e the angle of height relative to the base with the x axis)
        double dst = 0;
        Point top_vertex;
        for (auto& vertex: triangle) {
            const double dx = vertex.x-cx;
            const double dy = vertex.y-cy;
            const double curr_d = dx*dx + dy*dy;
            if (curr_d > dst) {
                dst = curr_d;
                top_vertex = vertex;
            }
        }

        // Store the position of the robot in the output
        x = cx;
        y = cy;

        // Compute the robot orientation
        const double dx = cx - top_vertex.x;
        const double dy = cy - top_vertex.y;
        theta = atan2(dy, dx);

        cv::Point cv_baricenter(x*scale, y*scale); // convert back m to px
        cv::Point cv_vertex(top_vertex.x*scale, top_vertex.y*scale); // convert back m to px
        #ifdef DEBUG_FINDROBOT
            // Draw over the image
            cv::line(contours_img, cv_baricenter, cv_vertex, cv::Scalar(0,255,0), 3);
            cv::circle(contours_img, cv_baricenter, 5, cv::Scalar(0,0,255), -1);
            cv::circle(contours_img, cv_vertex, 5, cv::Scalar(0,255,0), -1);
            cout << "(x, y, theta) = " << x << ", " << y << ", " << theta*180/M_PI << endl;
        #endif
    }

    #ifdef DEBUG_FINDROBOT
        cv::imshow("findRobot", contours_img);
        cv::waitKey(0);
    #endif

    return found;
}

/*!
* Find the arrival point
* Returns the baricenter of the gate polygon with the correct arrival angle
*/
void centerGate(const Polygon& gate, const Polygon& borders, double& x,
                 double& y, double& theta) {

    assert(gate.size() == 4);
    assert(borders.size() == 4);
    #ifdef DEBUG_GATE
        printf("---Center gate called---\n");
        printf("There are %zd points\n", gate.size());
    #endif

    ////   GATE BARICENTER
    double gc_x=0, gc_y=0;
    baricenter(gate,gc_x,gc_y);

    //Find shortest rectangle side between the first two
    Point shortest_p0;
    Point shortest_p1;
    double distance0 = pow((gate.at(0).x - gate.at(1).x), 2) +
                       pow((gate.at(0).y - gate.at(1).y), 2);
    double distance1 = pow((gate.at(1).x - gate.at(2).x), 2) +
                       pow((gate.at(1).y - gate.at(2).y), 2);
    if (distance0 < distance1) {
        shortest_p0.x = gate.at(0).x;
        shortest_p0.y = gate.at(0).y;
        shortest_p1.x = gate.at(1).x;
        shortest_p1.y = gate.at(1).y;
    } else {
        shortest_p0.x = gate.at(1).x;
        shortest_p0.y = gate.at(1).y;
        shortest_p1.x = gate.at(2).x;
        shortest_p1.y = gate.at(2).y;
    }
    ////   GATE ANGLE
    double angle;
    angle = atan2(shortest_p0.y-shortest_p1.y,shortest_p0.x-shortest_p1.x);
    if (angle < 0)
        angle += 2 * CV_PI;

    // Decide if the angle is the found one or the opposite (+180°) based on
    // the position of the gate
    double ac_x = 0, ac_y = 0;
    baricenter(borders,ac_x,ac_y);

    #ifdef DEBUG_GATE
        printf("Gate baricenter %f,%f\n",gc_x,gc_y);
        printf("Arena baricenter %f,%f\n",ac_x,ac_y);
    #endif

    if (gc_y > ac_y) {
        angle -= CV_PI;
        if (angle < 0)
            angle += 2 * CV_PI;
    }
    ////   Assign Output
    x = gc_x;
    y = gc_y;
    theta = angle;
}

// choose the curve with the arrival angle that minimizes the length    //TODO: check why this is not called by any function
dubins::Curve findBestAngle(double& th0, double& thf, double& x0, double& y0,
                            double& xf, double& yf, double& Kmax, int& pidx) {
    #define DEBUG_FINDBESTANGLE
#ifdef DEBUG_FINDBESTANGLE
    static int fba_counter = 0;
    fba_counter++;
    cout << "findBestAngle() called #" << fba_counter << endl;
#endif
    vector<double> angles;
    double best_length = 100000;
    dubins::Curve best_curve = dubins::Curve(0,0,0,0,0,0,0,0,0);

    // check start angle, end angle and angle in between (can be refined with more angles)
    angles.push_back(th0);
    angles.push_back(thf);
    angles.push_back((th0 + thf)/2);

    for (int i = 0; i < angles.size(); i++) {
        #ifdef DEBUG_FINDBESTANGLE
            cout << "dubins_shortest_path called from findBestAngle #" << fba_counter << endl;
        #endif
        dubins::Curve curve = dubins::dubins_shortest_path(x0, y0, th0, xf, yf, angles[i], Kmax, pidx);
        if (curve.L < best_length) {
            best_length = curve.L;
            thf = angles[i];
            best_curve = curve;
        }
    }

    return best_curve;
  }

bool isArcColliding(dubins::Arc a, Point pA, Point pB) {
    cv::Point2f center;
    float radius = abs(1 / a.k);
    bool clockwise;
    float xc, yc;

    if (a.k < 0) {
        xc = cos(a.th0 - M_PI/2) * radius; //(th0 is perpendicular to the radius)
        yc = sin(a.th0 - M_PI/2) * radius;

        clockwise = false;
    } else {
        xc = cos(a.th0 + M_PI/2) * radius;
        yc = sin(a.th0 + M_PI/2) * radius;

        clockwise = true;
    }

    center = cv::Point2f(xc + a.x0,yc + a.y0);

    float x1 = pA.x;
    float y1 = pA.y;
    float x2 = pB.x;
    float y2 = pB.y;

    float p1 = 2 * x1 * x2;
    float p2 = 2 * y1 * y2;
    float p3 = 2 * center.x * x1;
    float p4 = 2 * center.x * x2;
    float p5 = 2 * center.y * y1;
    float p6 = 2 * center.y * y2;

    // c1*t^2 + c2*t + c3 = 0
    float c1 = x1*x1 + x2*x2 - p1 + y1*y1 + y2*y2 - p2;
    float c2 = -2*x2*x2 + p1 - p3 + p4 - 2*y2*y2 + p2 - p5 + p6;
    float c3 = x2*x2 - p4 + center.x*center.x + y2*y2 - p6 + center.y*center.y - radius*radius;

    float delta = c2*c2 - 4 * c1 * c3; // calculate the delta of the equation
    vector<float> t_vector = {};

    if (delta < 0) {
        return false;
    } else if (delta*delta <= 0.000001) { // only one solution case
        float t1 = (-c2) / (2*c1);

        if (t1 >= 0 && t1 <= 1) {    // if t1 is in the t[0,1] interval the solution is in the segment
            t_vector.push_back(1-t1);
        }

    } else { // two solutions
        float t1 = (-c2 + sqrt(delta)) / (2*c1);
        float t2 = (-c2 - sqrt(delta)) / (2*c1);
        if (t1 >= 0 && t1 <= 1) { // if t1 is in the t[0,1] interval the solution is in the segment
            t_vector.push_back(1-t1);
        }
        if (t2 >= 0 && t2 <= 1) { // if t2 is in the t[0,1] interval the solution is in the segment
            t_vector.push_back(1-t2);
        }
    }

    for (unsigned int j = 0; j < t_vector.size(); j++) {

        float x_intersection = x1 + t_vector[j]*(x2 - x1);
        float y_intersection = y1 + t_vector[j]*(y2 - y1);

        float thetat = atan2((y_intersection - center.y),(x_intersection - center.x));
        float theta_start = atan2((pA.y - center.y),(pA.x - center.x));
        float theta_finish = atan2((pB.y - center.y),(pB.x - center.x));
        thetat = dubins::mod2pi(thetat);
        theta_start = dubins::mod2pi(theta_start);
        theta_finish = dubins::mod2pi(theta_finish);

        if (clockwise) {
            if (theta_finish >= theta_start && thetat >= theta_start && thetat <= theta_finish) {
                return true;
            } else if (theta_finish < theta_start && ((theta_start <= thetat) || (thetat <= theta_finish))) {
                return true;
            }
        } else {
            if (theta_finish >= theta_start && (thetat <= theta_start || thetat >= theta_finish)) {
                return true;
            } else if (theta_start > theta_finish && theta_finish <= thetat && thetat <= theta_start) {
                return true;
            }
        }

    }
    return false;
}

bool isSegmentColliding(Point a1, Point a2, Point p1, Point p2) {
    bool isSegColliding = false;
    float x1 = p1.x;
    float y1 = p1.y;
    float x2 = p2.x;
    float y2 = p2.y;

    float x3 = a1.x;
    float y3 = a1.y;
    float x4 = a2.x;
    float y4 = a2.y;

    float determinant = (x4 - x3) * (y1 - y2) - (x1 - x2) * (y4 - y3);

    // solutions if det != 0
    if (determinant != 0) {
        float t = ((y3 - y4)*(x1 - x3) +(x4 - x3)*(y1 - y3) ) / determinant;
        float u = ((y1 - y2)*(x1 - x3) +(x2 - x1)*(y1 - y3) ) / determinant;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            isSegColliding = true;
        } else {
            isSegColliding = false;
        }
    }

    return isSegColliding;
}

// approximate approach to check for collisions between arc and segment using discretized arc
bool isDiscretizedArcColliding(dubins::Arc& a, Point pA, Point pB) {
    Polygon d_arc;

    #ifdef DEBUG_COLLISION
        cv::Mat img = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255,255,255));
    #endif

    double remainingDelta = 0.0;
    double last_s = 0.0;
    vector<dubins::Position> res = a.discretizeArc(0.01, remainingDelta, last_s, true);

    for (const auto& pt: res) {
        d_arc.emplace_back(pt.x, pt.y);
    }

    #ifdef DEBUG_COLLISION
        for (int j = 0; j < d_arc.size()-1; j++)
        {
            cv::line(img, cv::Point(d_arc[j].x*debugImagesScale, d_arc[j].y*debugImagesScale),
                cv::Point(d_arc[j+1].x*debugImagesScale, d_arc[j+1].y*debugImagesScale), cv::Scalar(255,200,0),1); // draw the line
        }
    #endif

    #ifdef DEBUG_COLLISION
        cv::line(img, cv::Point(p[i].x*debugImagesScale, p[i].y*debugImagesScale),
            cv::Point(p[i+1].x*debugImagesScale, p[i+1].y*debugImagesScale), cv::Scalar(0,0,255),1); // draw the line
    #endif

    for (int j = 0; j < d_arc.size()-1; j++)
    {
        if (isSegmentColliding(d_arc[j], d_arc[j+1], pA, pB)) {

            #ifdef DEBUG_COLLISION
                cv::imshow("arc pol", img);
                cv::waitKey(0);
            #endif

            return true;
        }
    }

    return false;
}

bool isCollidingWithPolygon(dubins::Arc& a, Polygon p) {
    float k = a.k;

    // add first vertex again to check segment between first and last
    p.push_back(p[0]);

    // if k=0 a is a segment, else it is an arc

    // check each segment in polygon p
    for (int i = 0; i < p.size()-1; i++) {
        if (k == 0) {
            if (isSegmentColliding(Point(a.x0, a.y0), Point(a.xf, a.yf), p[i], p[i+1])) {
                return true;
            }
        } else {
            if (isDiscretizedArcColliding(a, p[i], p[i+1])) {
                return true;
            }

            /*if (isArcColliding(a, p[i], p[i+1])) {
                return true;
            }*/
        }
    }
    return false;
}

bool isCurveColliding(dubins::Curve& curve, const vector<Polygon>& obstacle_list) {
    for (Polygon p : obstacle_list) {
        if (isCollidingWithPolygon(curve.a1, p)) {
            return true;
        } else if (isCollidingWithPolygon(curve.a2, p)) {
            return true;
        } else if (isCollidingWithPolygon(curve.a3, p)) {
            return true;
        }
    }
    return false;
}

#ifdef DEBUG_DRAWCURVE

void drawDubinsArc(dubins::Arc& da) {

    cv::Point2f startf = cv::Point2f(da.x0*debugImagesScale, da.y0*debugImagesScale);
    cv::Point2f finishf = cv::Point2f(da.xf*debugImagesScale, da.yf*debugImagesScale);

    if (da.k == 0) {
        cv::line(dcImg, cv::Point(startf.x,startf.y), cv::Point(finishf.x,finishf.y), cv::Scalar(255,200,0),2); // draw the line
    } else {
        float radius = abs(1 / da.k)*debugImagesScale; // radius is 1/k
        cv::Point2f centerf;

        if (da.k < 0) {
            float xc = cos(da.th0 - M_PI/2) * radius;
            float yc = sin(da.th0 - M_PI/2) * radius;
            centerf = cv::Point(xc + da.x0*debugImagesScale,yc + da.y0*debugImagesScale);
        } else {
            float xc = cos(da.th0 + M_PI/2) * radius;
            float yc = sin(da.th0 + M_PI/2) * radius;
            centerf = cv::Point(xc + da.x0*debugImagesScale,yc + da.y0*debugImagesScale);
        }
        cv::Point start = cv::Point(startf.x,startf.y);
        cv::Point finish = cv::Point(finishf.x,finishf.y);

        float thetastart = atan2((startf.y - centerf.y),(startf.x - centerf.x));
        float thetafinish= atan2((finishf.y - centerf.y),(finishf.x - centerf.x));
        thetastart = dubins::mod2pi(thetastart);
        thetafinish = dubins::mod2pi(thetafinish);
        float passo = 0.0174533; // a degree in radiants

        if (da.k > 0) { // clockwise segment drawing
            for (unsigned int i = 0; 0.01 < ((dubins::mod2pi(thetastart + passo*i) - thetafinish)*(dubins::mod2pi(thetastart + passo*i) -thetafinish)); i++) {
                cv::Point startSegment = cv::Point(cos(thetastart + passo*i  )*radius + centerf.x,+ sin(thetastart + passo*i  )*radius + centerf.y);
                cv::Point finishSegment = cv::Point(cos(thetastart + passo*(i+1)  )*radius + centerf.x, sin(thetastart + passo*(i+1))*radius + centerf.y);
                cv::line(dcImg, startSegment, finishSegment, cv::Scalar(255,200,0),2); // draw the line
            }
        } else { // counter clockwise segment drawing
            for (unsigned int i = 0; 0.01 < ((dubins::mod2pi(thetastart - passo*i) - thetafinish)*(dubins::mod2pi(thetastart - passo*i) - thetafinish)); i++) {
                cv::Point startSegment = cv::Point(cos(thetastart - passo*i  )*radius + centerf.x,+ sin(thetastart - passo*i  )*radius + centerf.y);
                cv::Point finishSegment = cv::Point(cos(thetastart - passo*(i+1)  )*radius + centerf.x, sin(thetastart - passo*(i+1))*radius + centerf.y);
                cv::line(dcImg, startSegment, finishSegment, cv::Scalar(255,200,0),2); // draw the line
           }
        }

        cv::circle(dcImg,start,2,cv::Scalar(0,0,0),3);
        cv::circle(dcImg,finish,2,cv::Scalar(0,0,0),3);
    }
}

#endif

std::pair<bool,std::vector<dubins::Curve>> MDP(const std::vector<Point> &path,
                                               unsigned int startIdx, unsigned int arriveIdx,
                                               double startAngle, double arriveAngle,
                                               double& returnedLength, const double Kmax,
                                               const vector<Polygon>& obstacle_list) {
    /* ----------------------------- PARAMETERS ----------------------------- */
    const unsigned short NUM_ANGLES = 4;   // Number of angles to test for each free point
    const bool MDP_VERBOSE = false;        // Setting this to true prints additional info for debug (Warning: it can get verbose)
    const bool USE_ANGLE_HEURISTIC = true; // Setting this to true tries to improve the way that free angles are chosen

    /* ------------------------- RECURSIVE ALGORITHM ------------------------ */
    if (MDP_VERBOSE) cout << "called with indexes " << startIdx << ", "<< arriveIdx << endl;

    // RECURSION ERROR CASES:
    if (arriveIdx == startIdx)
        throw std::invalid_argument("Error: arriveIdx cannot be equal to startIdx");
    if (arriveIdx < startIdx)
        throw std::invalid_argument("Error: arriveIdx cannot be smaller than startIdx");

    // RECURSION BASE CASE 1: Called on two points, the function computes the connecting Dubins path
    //           o---o
    // nodes:    1   2
    // segments:   A
    if (arriveIdx-startIdx == 1) {
        if (MDP_VERBOSE) cout << "MDP(): Recursion BASE Case 1" << endl;
        // Compute the curve for segment A and check for collisions
        int pidx = 0;
        double x1 = path[startIdx].x;
        double y1 = path[startIdx].y;
        double theta1 = startAngle;
        double x2 = path[arriveIdx].x;
        double y2 = path[arriveIdx].y;
        double theta2 = arriveAngle;
        dubins::Curve curve = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, Kmax, pidx);
        bool isColliding = isCurveColliding(curve, obstacle_list);

        returnedLength = isColliding ? std::numeric_limits<double>::max() : curve.L;
        std::vector<dubins::Curve> multipointPath(path.size()-1);    // segments are one less than the num of points
        if (!isColliding)
            multipointPath.at(startIdx) = curve; //ERRORE?

        return std::make_pair(!isColliding,multipointPath);  // Return true if the path does not collide
    }

    // RECURSION BASE CASE 2: Called on three points it checks for NUM_ANGLES for the middle point
    //            o---o---o
    // nodes:     1   2   3
    // segments:    A   B
    if (arriveIdx-startIdx == 2) {
        if (MDP_VERBOSE) cout << "MDP(): Recursion BASE Case 2" << endl;
        std::pair<dubins::Curve,dubins::Curve> bestCurves;
        double bestLength =  std::numeric_limits<double>::max();
        bool allAnglesResultInCollision = true; // If no angle choice provides a non-colliding path, the call must fail

        for(int i = 0; i < NUM_ANGLES; ++i) {
            double alpha_middlepoint = (2*M_PI)/NUM_ANGLES * i;
            if (USE_ANGLE_HEURISTIC)
                alpha_middlepoint += (startAngle + arriveAngle)/2;   // adding the average of the other angles should improve the angle choice

            double x1, y1, theta1, x2, y2, theta2;
            int pidx;
            // Compute the curve for segment A and check for collisions
            // A: starts from startIdx, angle: startAngle | ends in startIdx+1, angle: alpha_middlepoint
            pidx = 0;
            x1 = path[startIdx].x;
            y1 = path[startIdx].y;
            theta1 = startAngle;
            x2 = path[startIdx+1].x;
            y2 = path[startIdx+1].y;
            theta2 = alpha_middlepoint;
            dubins::Curve curveA = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, Kmax, pidx);
            bool isAColliding = isCurveColliding(curveA, obstacle_list);
            // Compute the curve for segment B and check for collisions
            // B: starts from arriveIdx-1, angle: alpha_middlepoint | ends in arriveIdx, angle: arriveAngle
            pidx = 0;
            x1 = path[arriveIdx-1].x;
            y1 = path[arriveIdx-1].y;
            theta1 = alpha_middlepoint;
            x2 = path[arriveIdx].x;
            y2 = path[arriveIdx].y;
            theta2 = arriveAngle;
            dubins::Curve curveB = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, Kmax, pidx);
            bool isBColliding = isCurveColliding(curveB, obstacle_list);

            double lengthAB = curveA.L + curveB.L;

            if ((!isAColliding) && (!isBColliding) && (lengthAB < bestLength)) {
                allAnglesResultInCollision = false;
                bestLength = lengthAB;
                bestCurves = std::make_pair(curveA,curveB);
            }
        }
        std::vector<dubins::Curve> multipointPath(path.size()-1);    // segments are one less than the num of points
        if (!allAnglesResultInCollision) {    //push best curves in path IF !allAnglesResultInCollision
            assert(bestLength == bestCurves.first.L + bestCurves.second.L);    // sanity check
            multipointPath.at(startIdx) = bestCurves.first;
            multipointPath.at(arriveIdx-1) = bestCurves.second;
        }

        returnedLength = allAnglesResultInCollision ? std::numeric_limits<double>::max() : bestLength;  //Return best length
        return std::make_pair(!allAnglesResultInCollision,multipointPath);  // Return true if the path does not collide
    }

    // RECURSION STEP: for more than 3 points (2 segments)
    //              o---o [...] o---o
    // nodes:       1   2      n-1  n
    // segments:      A           B
    // the solution corresponds to the sum of the initial segment, the recursive
    // path for indexes in the middle and the final segment.
    // The initial and final angles are fixed (exit angle from node 1 and
    // arrival angle in node n. The arrival angle in node 2 and the exit angle
    // from node n-1 are free and they are chosen from a pool of NUM_ANGLES
    // according to which ones correspond to the shortest non-colliding path.
    // NOTE: the arrival angle for node 2 is called alpha_first and the exit
    // angle from node n-1 is called alpha_second
    if (MDP_VERBOSE) cout << "MDP(): RECURSIVE step" << endl;

    std::pair<dubins::Curve,dubins::Curve> bestCurves;  // best dubins curves yet
    double bestLength = std::numeric_limits<double>::max();
    std::vector<dubins::Curve> bestRecursivePath;
    bool allAnglesResultInCollision = true; // If no angle choice provides a non-colliding path, the call must fail

    for(int i = 0; i < NUM_ANGLES; ++i) {
        for(int j = 0; j < NUM_ANGLES; ++j) {
            double alpha_first = (2*M_PI)/NUM_ANGLES * i;
            double alpha_second = (2*M_PI)/NUM_ANGLES * j;

            if (USE_ANGLE_HEURISTIC) {
                alpha_first += startAngle;
                alpha_second += arriveAngle;
            }

            double x1, y1, theta1, x2, y2, theta2;
            int pidx;
            // Compute the curve for segment A and check for collisions
            // A: starts from startIdx, angle: startAngle | ends in startIdx+1, angle: alpha_first
            pidx = 0;
            x1 = path[startIdx].x;
            y1 = path[startIdx].y;
            theta1 = startAngle;
            x2 = path[startIdx+1].x;
            y2 = path[startIdx+1].y;
            theta2 = alpha_first;
            dubins::Curve curveA = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, Kmax, pidx);
            bool isAColliding = isCurveColliding(curveA, obstacle_list);
            // Compute the curve for segment B and check for collisions
            // B: starts from arriveIdx-1, angle: alpha_second | ends in arriveIdx, angle: arriveAngle
            pidx = 0;
            x1 = path[arriveIdx-1].x;
            y1 = path[arriveIdx-1].y;
            theta1 = alpha_second;
            x2 = path[arriveIdx].x;
            y2 = path[arriveIdx].y;
            theta2 = arriveAngle;
            dubins::Curve curveB = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, Kmax, pidx);
            bool isBColliding = isCurveColliding(curveB, obstacle_list);

            double recursivelyReturnedLength = -1;

            bool result = false;
            std::vector<dubins::Curve> recursiveReturnedPath;
            if ((!isAColliding) && (!isBColliding)) {
                std::pair<bool,std::vector<dubins::Curve>> tuple;
                tuple = MDP(path, startIdx+1,arriveIdx-1,
                            alpha_first,alpha_second,
                            recursivelyReturnedLength,
                            Kmax, obstacle_list);
                result = tuple.first;
                recursiveReturnedPath = tuple.second;
            }

            double totLength = curveA.L + recursivelyReturnedLength + curveB.L;

            if (result)
                assert(recursivelyReturnedLength != std::numeric_limits<double>::max());    // sanity check
            if ((result) && (totLength < bestLength)) {
                allAnglesResultInCollision = false;
                bestLength = totLength;
                bestCurves = std::make_pair(curveA,curveB);
                bestRecursivePath = recursiveReturnedPath;
            }
        }
    }

    if (!allAnglesResultInCollision) {    //push best curves in path IF !allAnglesResultInCollision
        assert(bestLength > bestCurves.first.L + bestCurves.second.L);    // sanity check
        bestRecursivePath.at(startIdx) = bestCurves.first;
        bestRecursivePath.at(arriveIdx-1) = bestCurves.second;
    }

    returnedLength = allAnglesResultInCollision ? std::numeric_limits<double>::max() : bestLength;  //Return best length
    return std::make_pair(!allAnglesResultInCollision,bestRecursivePath);  // Return true if the path does not collide
}

bool isPathColliding(vector<Point> vertices, vector<Polygon> obstacle_list) {
    for (int i = 1; i < vertices.size(); ++i) {
        double x0, y0, xf, yf;
        x0 = vertices[i-1].x;
        y0 = vertices[i-1].y;
        xf = vertices[i].x;
        yf = vertices[i].y;

        for (Polygon p : obstacle_list) {
            for (int k = 0; k < p.size(); k++) {
                bool collision = false;
                if (k == p.size()-1)
                    collision = isSegmentColliding(Point(x0,y0), Point(xf,yf), p[k], p[0]);
                else
                    collision = isSegmentColliding(Point(x0,y0), Point(xf,yf), p[k], p[k+1]);

                if (collision)
                    return true;
            }
        }
    }
    return false;
}

bool pathSmoothing(int start_index, int finish_index, vector<Point> vertices,
                   vector<Polygon> obstacle_list, vector<Point>& short_path) {

    // TODO: recursion missing STOP condition, eg. start_index == finish_index then return false

    bool collision = false;
    double x0, y0, xf, yf;

    x0 = vertices[start_index].x;
    y0 = vertices[start_index].y;

    xf = vertices[finish_index].x;
    yf = vertices[finish_index].y;

    for (Polygon p : obstacle_list) {
        for (int k = 0; k < p.size(); k++) {
            if (k == p.size()-1) {
                collision = isSegmentColliding(Point(x0,y0), Point(xf,yf), p[k], p[0]);
            } else {
                collision = isSegmentColliding(Point(x0,y0), Point(xf,yf), p[k], p[k+1]);
            }

            if (collision) {
              break;
            }
        }
        if (collision) {
            break;
        }
    }

    if (!collision) {
        short_path.push_back(vertices[finish_index]);
        return true;
    } else {
        int mid_point = (start_index + finish_index)/2;

        if (finish_index-start_index > 1) {
            bool r1 = pathSmoothing(start_index, mid_point, vertices, obstacle_list, short_path);
            bool r2 = pathSmoothing(mid_point, finish_index, vertices, obstacle_list, short_path);

            return r1 && r2;
        } else {
            return false;
        }
    }
}

vector<Point> RRTplanner(const Polygon& borders, const vector<Polygon>& obstacle_list,
                  const float x0, const float y0, const float xf, const float yf,
                  const string& config_folder) {

    //
    // write the problem parameters to a file that will be fed to a planning lib
    //

    const string vcd_dir = config_folder + "/../src/path-planning";
    ofstream output(vcd_dir + "/i.txt");
    if (!output.is_open()) {
        throw runtime_error("Cannot write file: " + vcd_dir + "/i.txt");
    }

    #ifdef DEBUG_RRT
        printf("Writing problem parameters to file\n");
        printf("Measure are upscaled by a scale factor of: %d\n",pythonUpscale);
    #endif

    // write borders on the first line
    for (int i = 0; i < borders.size(); i++) {
        if (i < borders.size()-1) {
            output << "(" << int(borders[i].x*pythonUpscale) << "," << int(borders[i].y * pythonUpscale) << "),";
        } else {
            output << "(" << int(borders[i].x*pythonUpscale) << "," << int(borders[i].y * pythonUpscale) << ")" << endl;
        }
    }

    //  write vertices on the second line
    for (int i = 0; i < obstacle_list.size(); i++) {
        //add each vertex in (x,y) format
        for (int j = 0; j < obstacle_list[i].size(); j++) {
            int pt_x = int(obstacle_list[i][j].x * pythonUpscale);
            int pt_y = int(obstacle_list[i][j].y * pythonUpscale);

            if (j < obstacle_list[i].size() - 1) {
                output << "(" << pt_x << ", " << pt_y << "), ";
            } else {
                output << "(" << pt_x << ", " << pt_y << ")" << endl;
            }
        }
    }

    //write source and destination on the third line
    output << "(" << int(x0*pythonUpscale) << "," << int(y0*pythonUpscale) << "),(" << int(xf*pythonUpscale) << "," << int(yf*pythonUpscale) << ")";

    output.close();

    //
    // The text file has ben prepared, now the external plan library is called
    //

    // prepare script command
    string cmd = "python " + vcd_dir + "/rrt.py -in " + vcd_dir + "/i.txt -out " + vcd_dir + "/output.txt";
    char str[cmd.size()+1];
    strcpy(str, cmd.c_str());
    // call library script
    system(str);

    //
    // Read the resulting path
    //

    // read vertices and path from output.txt
    ifstream input(vcd_dir + "/output.txt");
    vector<Point> vertices;

    bool path_not_found = false;
    if (input.is_open()) {
        string line;
        while (getline(input,line)) {
            input >> line;
            istringstream ss(line);
            string token;

            float v1, v2;
            int count = 0;

            while (getline(ss, token, ',')) {
                if (stoi(token) == -1) {
                    path_not_found = true;
                } else {
                    if (count%2 == 0) {
                        v1 = stod(token);
                    } else {
                        v2 = stod(token);
                        vertices.push_back(Point(v1/pythonUpscale,v2/pythonUpscale));   //Scaling back points using the scale factor for the library
                    }
                    count++;
                }
            }
        }
    }

    if (path_not_found) {
        throw runtime_error("Path not found!");
    }

    return vertices;
}

vector<Point> completeSmoothing(const vector<Point>& path,const vector<Polygon>& obstacle_list) { //TODO: change name
    vector<Point> smoothedPath; //TODO: change name
    smoothedPath.push_back(path[0]);
    bool is_path_smoothed = pathSmoothing(0, path.size()-1, path, obstacle_list, smoothedPath);

    if (is_path_smoothed) {
        #ifdef DEBUG_PATH_SMOOTHING
            cout << "\t>Path shortened ONCE  (size: " << smoothedPath.size() << ")" << endl;
        #endif
        bool additional_shortening;
        do{
            vector<Point> shorter_path;
            shorter_path.push_back(path[0]);
            bool success = pathSmoothing(0, smoothedPath.size()-1, smoothedPath, obstacle_list, shorter_path);
            additional_shortening = success && (shorter_path.size() < smoothedPath.size());
            if (additional_shortening) {
                #ifdef DEBUG_PATH_SMOOTHING
                    cout << "\t>Path shortened AGAIN (size: " << smoothedPath.size() << "->" << shorter_path.size() << ")" << endl;
                #endif
                smoothedPath = shorter_path;
            }
        }while(additional_shortening);

        assert(smoothedPath.front() == path.front());
        assert(smoothedPath.back() == path.back());

        {
            // additional iteration on reversed path
            vector<Point> shorter_path;
            std::reverse(smoothedPath.begin(), smoothedPath.end());
            shorter_path.push_back(path[path.size()-1]);
            bool reverse_smoothing = pathSmoothing(0, smoothedPath.size()-1, smoothedPath, obstacle_list, shorter_path);
            if (reverse_smoothing) {
                #ifdef DEBUG_PATH_SMOOTHING
                    cout << "Reverse smoothing SUCCESS" << endl;
                    cout << "\t>Path shortened AGAIN (size: " << smoothedPath.size() << "->" << shorter_path.size() << ")" << endl;
                #endif
                std::reverse(shorter_path.begin(), shorter_path.end());
                smoothedPath = shorter_path;
            }else{
                #ifdef DEBUG_PATH_SMOOTHING
                    cout << "Reverse smoothing FAIL" << endl;
                #endif
            }
        }

        assert(smoothedPath.front() == path.front());
        assert(smoothedPath.back() == path.back());
        return smoothedPath;

    }else{
        #ifdef DEBUG_PATH_SMOOTHING
            cout << "Path smoothing FAILED! Collision detected" << endl;
        #endif
        throw logic_error("ERROR: path cannot be shortened");
        // TODO: Thie case might need to be handled in a better way:
        // This occurs often if the scaling factor for the python script is too
        // low. That causes approximation errors that result in RRT paths that
        // do not collide according to the script, but collide accorting to the
        // actual computations.

        // However, the scaling determines the granularity of the algorithm
        // (high scaling, longer output path) which means that the script could
        // hit its ITERATION LIMIT. This along with the fact that it's slower.
        // Either we handle the iteration limit case by taking the partial path
        // provided and launching other instances of the script to complete it
        // OR we go for a good floating point RRT library (elegant and fast).
        // (Maybe http://ompl.kavrakilab.org/planners.html)
    }
}

bool planPath(const Polygon& borders, const vector<Polygon>& obstacle_list,
              const vector<pair<int,Polygon>>& victim_list, const Polygon& gate,
              const float x, const float y, const float theta, Path& path,
              const string& config_folder) {

    #ifdef DEBUG_PLANPATH
        printf("--------PLANNING WAS CALLED--------\n");
        fflush(stdout);
    #endif

    double xf, yf, thf;                   // Endpoint
    centerGate(gate,borders,xf,yf,thf);  // Endpoint computation
    const double Kmax = 10.0;                   // Maximum curvature
    const double path_res = 0.01;               // Path resolution (sampling)

    //
    // Sort Victims by ID
    //

    vector<pair<int,Polygon>> orderedVictimList = victim_list;
    sort(orderedVictimList.begin(), orderedVictimList.end(), [](const pair<int,Polygon>& lhs, const pair<int,Polygon>& rhs) {
      return lhs.first < rhs.first;
    });

    //
    // Create a vector of crucial points (start, victims, end);
    //
    vector<Point> pathObjectives;
    pathObjectives.push_back(Point(x,y));              // push initial point
    for(const pair<int,Polygon>& victim : orderedVictimList)
        pathObjectives.push_back(baricenter(victim.second));  //push each victim center
    pathObjectives.push_back(Point(xf,yf));            // push final point

    //
    // Call a path planner for each segment to plan
    //

    vector<Point> vertices;
    vertices.push_back(Point(x,y));
    vector<Point> short_path;
    short_path.push_back(Point(x,y));
    for(int i = 1; i < pathObjectives.size(); ++i) {
        #ifdef DEBUG_PLANPATH
            cout << "Planning segment " << i << "/" << pathObjectives.size() << endl;
        #endif
        //
        // PLANNING Step 1: Call RRT planner
        //
        float x1,y1,x2,y2;
        x1 = pathObjectives[i-1].x;
        y1 = pathObjectives[i-1].y;
        x2 = pathObjectives[i].x;
        y2 = pathObjectives[i].y;
        vector<Point> partialPath = RRTplanner(borders,obstacle_list,x1,y1,x2,y2,config_folder);    //TODO: change vertices name with something more appropriate?
        vertices.insert(vertices.end(),partialPath.begin()+1,partialPath.end());    // begin()+1 not to repeat points
        assert(!isPathColliding(partialPath, obstacle_list));  // If the rrt path collides there is an error in the python script or conversion

        //
        // PLANNING Step 2: Smoothing/Shortcutting
        //
        vector<Point> partialShortPath = completeSmoothing(partialPath,obstacle_list);
        short_path.insert(short_path.end(), partialShortPath.begin()+1, partialShortPath.end());    // begin()+1 not to repeat points
    }
    assert(short_path.front() == vertices.front());
    assert(short_path.back() == vertices.back());

    #ifdef DEBUG_PLANPATH
        cout << "------------------------------------------------------------" << endl;
        cout << "> Planning Step 1: planned RRT path ("<< vertices.size() <<" steps)" << endl;
        cout << "------------------------------------------------------------" << endl;
        cout << "> Planning Step 2: smoothed path ("<< short_path.size() <<" steps)" << endl;
        cout << "------------------------------------------------------------" << endl;
    #endif

#ifdef DEBUG_DRAWCURVE
    //
    //  Draw Curves to debug errors
    //

    const bool VERBOSE_DEBUG_DRAWCURVE = false;

    // draw borders
    for (int i = 0; i < borders.size(); i++)
        cv::line(dcImg, cv::Point(borders[i].x*debugImagesScale, borders[i].y*debugImagesScale), cv::Point(borders[i+1].x*debugImagesScale,borders[i+1].y*debugImagesScale), cv::Scalar(0,0,0),3); // draw the line
    // draw original path
    for (int i = 0; i < vertices.size(); i++) {
        cv::circle(dcImg, cv::Point(vertices[i].x*debugImagesScale, vertices[i].y*debugImagesScale), 2, cv::Scalar(0,0,0),CV_FILLED);
        if (VERBOSE_DEBUG_DRAWCURVE) cout << to_string(i) << ": " << to_string(vertices[i].x) << "," << vertices[i].y << endl;
        if (i > 0)
            cv::line(dcImg, cv::Point(vertices[i-1].x*debugImagesScale, vertices[i-1].y*debugImagesScale),
                     cv::Point(vertices[i].x*debugImagesScale, vertices[i].y*debugImagesScale), cv::Scalar(255,0,0),2);
    }
    /*
    //Draw Smothed path Dots and lines
    for (int i = 0; i < short_path.size(); i++) {
        cv::circle(dcImg, cv::Point(short_path[i].x*debugImagesScale, short_path[i].y*debugImagesScale), 2, cv::Scalar(255,235,0),CV_FILLED);
        if (VERBOSE_DEBUG_DRAWCURVE) cout << to_string(short_path[i].x) << "," << short_path[i].y << endl;
        if (i > 0)
            cv::line(dcImg, cv::Point(short_path[i-1].x*debugImagesScale, short_path[i-1].y*debugImagesScale),
                     cv::Point(short_path[i].x*debugImagesScale, short_path[i].y*debugImagesScale), cv::Scalar(255,235,0),2);
    }
    */
    //Draw obstacles
    for (const Polygon &pol : obstacle_list) {
        for (int i=1; i<pol.size(); ++i) {
            cv::line(dcImg, cv::Point(pol[i-1].x*debugImagesScale, pol[i-1].y*debugImagesScale),
                     cv::Point(pol[i].x*debugImagesScale, pol[i].y*debugImagesScale), cv::Scalar(0,0,255),2);
        }
        cv::line(dcImg, cv::Point(pol[0].x*debugImagesScale, pol[0].y*debugImagesScale),
                     cv::Point(pol[pol.size()-1].x*debugImagesScale, pol[pol.size()-1].y*debugImagesScale), cv::Scalar(0,0,255),2);
    }
#endif

    // Startpoint
    double x0 = short_path[0].x;
    double y0 = short_path[0].y;
    double th0 = theta;

    // add borders for collision check
    vector<Polygon> boundaries = obstacle_list;
    boundaries.push_back(borders);

    double returnedLength = 0;  // unused here, just for recursion
    #ifdef DEBUG_PLANPATH
        cout << "Computing Multi Point Dubins path..." << endl;
    #endif
    std::pair<bool,vector<dubins::Curve>> multipointResult;
    multipointResult = MDP(short_path,
                           0, short_path.size()-1,   // start and arrival indexes
                           th0, thf,                 // start and arrive angles
                           returnedLength, Kmax,
                           boundaries);
    bool path_planned = multipointResult.first;
    vector<dubins::Curve> multipointPath = multipointResult.second;

#ifdef DEBUG_DRAWCURVE

    for (int i = 0; i < multipointPath.size(); i++)
    {
        drawDubinsArc(multipointPath[i].a1);
        drawDubinsArc(multipointPath[i].a2);
        drawDubinsArc(multipointPath[i].a3);
    }

    cv::flip(dcImg, dcImg, 0);
    cv::imshow("Curves",dcImg); ///////////////////
    cv::waitKey(0);

#endif

    if (path_planned) {
        #ifdef DEBUG_PLANPATH
            cout << "> Planning Step 3: Multipoint dubins curve planned successfully" << endl;
            cout << "------------------------------------------------------------" << endl;
        #endif
    } else {
        #ifdef DEBUG_PLANPATH
            cout << "> Planning Step 3: Multipoint dubins curve planning" << endl;
            cout << "------------------------------------------------------------" << endl;
        #endif
        throw runtime_error("Could NOT plan path!");
    }

    vector<Pose> points;

    for (int i = 0; i < multipointPath.size(); i++) {
        // Sample the curve with resolution @path_res
        vector<dubins::Position> res = multipointPath[i].discretizeSingleCurve(path_res);   //TODO: change to discretizeCurve() and manage carry values (remainders of the discretization used to have equal distance between points)

        // Path conversion into compatible output representation
        for (dubins::Position p : res) {
            Pose pose(p.s,p.x,p.y,p.th,p.k);
            points.push_back(pose);
        }
    }

    //Set output
    path.setPoints(points);

    return true;
  }
}
