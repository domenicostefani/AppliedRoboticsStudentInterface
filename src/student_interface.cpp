#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "dubins.hpp"
#include "hsv_panel.hpp"

#include <experimental/filesystem>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <math.h>

#define AUTO_CORNER_DETECTION false

// #define COLOR_RANGE_DEBUG // Prints info about HSV ranges used for colors
#define DUBINS_DEBUG false
//#define IMSHOW_DEBUG

using namespace std;

const string COLOR_CONFIG_FILE = "/color_parameters.config";
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

namespace student {
int i = 0;
int scale = 200;

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
        filename += to_string(i++);
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
        #ifdef COLOR_CONFIG_DEBUG
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

            #ifdef COLOR_CONFIG_DEBUG
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
    // #define COLOR_CONFIG_DEBUG

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
    // #define ESTRINISIC_CALIB_DEBUG
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

    #ifdef ESTRINISIC_CALIB_DEBUG
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
    #ifdef COLOR_RANGE_DEBUG
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
    #ifdef COLOR_RANGE_DEBUG
        printf("Using RED bound 2 (%d,%d,%d)-(%d,%d,%d)\n", lowH2, lowS2,  lowV2, highH2, highS2, highV2);
    #endif
    cv::inRange(hsv_img, cv::Scalar(lowH1, lowS1, lowV1), cv::Scalar(highH1, highS1, highV1), lower_red_hue_range);
    cv::inRange(hsv_img, cv::Scalar(lowH2, lowS2, lowV2), cv::Scalar(highH2, highS2, highV2), upper_red_hue_range);

    // Now we can combine the 2 masks
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    //This can reduce false positives
    // cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2); //TODO: keep or remove? decide


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

    #ifdef IMSHOW_DEBUG
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

    #ifdef COLOR_RANGE_DEBUG
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

    #ifdef IMSHOW_DEBUG
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

    #ifdef COLOR_RANGE_DEBUG
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

    #ifdef IMSHOW_DEBUG
        cv::imshow("findVictims", contours_img);
        cv::waitKey(0);
    #endif
    // TEMPLATE MATCHING

    cv::Mat img;
    cv::cvtColor(hsv_img, img, cv::COLOR_HSV2BGR);

    // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
    cv::Mat green_mask_inv, filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255,255,255));
    cv::bitwise_not(green_mask, green_mask_inv);

    #ifdef IMSHOW_DEBUG
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
        #ifdef IMSHOW_DEBUG
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

bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle,
               double& x, double& y, double& theta,
               const string& config_folder) {
    // #define FIND_ROBOT_DEBUG_PLOT
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

    #ifdef COLOR_RANGE_DEBUG
        printf("Using BLUE bound  (%d,%d,%d)-(%d,%d,%d)\n", lowH, lowS, lowV, highH, highS, highV);
    #endif
    cv::Mat blue_mask;
    cv::inRange(hsv_img, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), blue_mask);

    // remove noise with opening operation
    //cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

    // find robot contours and approximate to triangle
    vector<vector<cv::Point>> contours;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    #ifdef FIND_ROBOT_DEBUG_PLOT // do this only if FIND_DEBUG_PLOT is defined
        //cv::imshow("findRobotHsv", hsv_img);          //TODO: keep or remove? decide
        //cv::imshow("findRobotMask", blue_mask);
        cv::Mat contours_img;
        contours_img = img_in.clone();
        //cv::drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
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
        #ifdef FIND_ROBOT_DEBUG_PLOT
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
        #ifdef FIND_ROBOT_DEBUG_PLOT
            // Draw over the image
            cv::line(contours_img, cv_baricenter, cv_vertex, cv::Scalar(0,255,0), 3);
            cv::circle(contours_img, cv_baricenter, 5, cv::Scalar(0,0,255), -1);
            cv::circle(contours_img, cv_vertex, 5, cv::Scalar(0,255,0), -1);
            cout << "(x, y, theta) = " << x << ", " << y << ", " << theta*180/M_PI << endl;
        #endif
    }

    #ifdef IMSHOW_DEBUG
        cv::imshow("findRobot", contours_img);
        cv::waitKey(0);
    #endif

    return found;
}

/*!
* Find the arrival point
* Returns the baricenter of the gate polygon with the correct arrival angle
*/
void center_gate(const Polygon& gate, const Polygon& borders, double& x,
                 double& y, double& theta) {

    assert(gate.size() == 4);
    assert(borders.size() == 4);
    #ifdef GATE_DEBUG
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

    #ifdef GATE_DEBUG
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

// choose the curve with the arrival angle that minimizes the length
dubins::Curve findBestAngle(double& th0, double& thf, double& x0, double& y0,
                            double& xf, double& yf, double& Kmax, int& pidx) {

    vector<double> angles;
    double best_length = 100000;
    dubins::Curve best_curve = dubins::Curve(0,0,0,0,0,0,0,0,0);

    // check start angle, end angle and angle in between (can be refined with more angles)
    angles.push_back(th0);
    angles.push_back(thf);
    angles.push_back((th0 + thf)/2);

    for (int i = 0; i < angles.size(); i++) {
        dubins::Curve curve = dubins::dubins_shortest_path(x0, y0, th0, xf, yf, angles[i], Kmax, pidx);
        if (curve.L < best_length) {
            best_length = curve.L;
            thf = angles[i];
            best_curve = curve;
        }
    }

    return best_curve;
  }

bool isArcColliding(cv::Point2f center, bool clockwise, float radius,
                    dubins::Arc a, Point pA, Point pB) {
    float x1 = pA.x;
    float y1 = pA.y;
    float x2 = pB.x;
    float y2 = pB.y;

    float p1 = 2 * x1 * x2; // intermediate variables
    float p2 = 2 * y1 * y2;
    float p3 = 2 * center.x * x1;
    float p4 = 2 * center.x * x2;
    float p5 = 2 * center.y * y1;
    float p6 = 2 * center.y * y2;

    // c1, c2 and c3 are the coefficients of the equation c1*t^2 + c2*t +c3 = 0
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

bool isColliding(dubins::Arc& a, Polygon p) {
    float k = a.k;
    bool isArc;
    cv::Point2f p1 = cv::Point2f(a.x0, a.y0);
    cv::Point2f p2 = cv::Point2f(a.xf, a.yf);
    cv::Point2f center;
    float radius;
    bool clockwise;

    // if k=0, a is a segment, else it is an arc
    if (k == 0) {
        isArc = false;
    } else {
        radius = abs(1 / k);
        if (a.k < 0) {
            float xc = cos(a.th0 - M_PI/2) * radius; // xc = cos(th0 - Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin(a.th0 - M_PI/2) * radius; // yc = sin(th0 - Pi/2) * radius
            center = cv::Point2f(xc + a.x0,yc + a.y0);
            clockwise = false;
        } else {
            float xc = cos(a.th0 + M_PI/2) * radius; // xc = cos(th0 + Pi/2) * radius because th0 is perpendicular to the radius
            float yc = sin(a.th0 + M_PI/2) * radius; // yc = sin(th0 + Pi/2) * radius
            center = cv::Point2f(xc + a.x0,yc + a.y0);
            clockwise = true;
        }

        isArc = true;
    }

    // add first vertex again to check segment between first and last
    p.push_back(p[0]);

    for (int i = 0; i < p.size()-1; i++) {
        if (isArc) {
            if (isArcColliding(center, clockwise, radius, a, p[i], p[i+1])) {
                return true;
            }
        } else {
            if (isSegmentColliding(Point(a.x0, a.y0), Point(a.xf, a.yf), p[i], p[i+1])) {
                return true;
            }
        }
    }
    return false;
}

bool curveCollision(dubins::Curve& curve, const vector<Polygon>& obstacle_list) {
    for (Polygon p : obstacle_list) {
        if (isColliding(curve.a1, p)) {
            return true;
        } else if (isColliding(curve.a2, p)) {
            return true;
        } else if (isColliding(curve.a3, p)) {
            return true;
        }
    }
    return false;
}

//-------------------DRAWING DUBINS CURVES-------------------

cv::Mat img = cv::Mat(200, 250, CV_8UC3, cv::Scalar(0,0,0));

// Method that draws a dubins arc
void drawDubinsArc(dubins::Arc& da) {

    cv::Point2f startf = cv::Point2f(da.x0*scale, da.y0*scale);
    cv::Point2f finishf = cv::Point2f(da.xf*scale, da.yf*scale);

    if (da.k == 0) {
        cv::line(img, cv::Point(startf.x,startf.y), cv::Point(finishf.x,finishf.y), cv::Scalar(0,235,0),1); // draw the line
    } else {
        float radius = abs(1 / da.k)*scale; // radius is 1/k
        cv::Point2f centerf;

        if (da.k < 0) {
            float xc = cos(da.th0 - M_PI/2) * radius;
            float yc = sin(da.th0 - M_PI/2) * radius;
            centerf = cv::Point(xc + da.x0*scale,yc + da.y0*scale);
        } else {
            float xc = cos(da.th0 + M_PI/2) * radius;
            float yc = sin(da.th0 + M_PI/2) * radius;
            centerf = cv::Point(xc + da.x0*scale,yc + da.y0*scale);
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
                cv::line(img, startSegment, finishSegment, cv::Scalar(0,255,255),1); // draw the line
            }
        } else { // counter clockwise segment drawing
            for (unsigned int i = 0; 0.01 < ((dubins::mod2pi(thetastart - passo*i) - thetafinish)*(dubins::mod2pi(thetastart - passo*i) - thetafinish)); i++) {
                cv::Point startSegment = cv::Point(cos(thetastart - passo*i  )*radius + centerf.x,+ sin(thetastart - passo*i  )*radius + centerf.y);
                cv::Point finishSegment = cv::Point(cos(thetastart - passo*(i+1)  )*radius + centerf.x, sin(thetastart - passo*(i+1))*radius + centerf.y);
                cv::line(img, startSegment, finishSegment, cv::Scalar(0,255,255),1); // draw the line
           }
        }

        cv::circle(img,start,2,cv::Scalar(0,255,255),1);
        cv::circle(img,finish,2,cv::Scalar(0,255,255),1);
    }
}

//-------------------end DRAWING DUBINS CURVES-------------------

/*  TODO: keep or remove? decide
// TODO: fix wrong thf angle
//
// int c = 0;

bool recursiveMDP(int i, int j, double th0, double thf, vector<Point>& short_path, vector<dubins::Curve>& multipoint_dubins_path, double& Kmax, int& pidx, const vector<Polygon>& obstacle_list) {

    //c++;
    //if (c > 500) {
    //  return false;
    //}

    double x0 = short_path[i].x;
    double y0 = short_path[i].y;

    double xf = short_path[j].x;
    double yf = short_path[j].y;

    dubins::Curve curve = dubins::Curve(0,0,0,0,0,0,0,0,0);
    bool collision = true;
    double alpha = th0;

    for (int i = 0; i < 16; i++) {
        alpha += 6.28/16*i;
        curve = dubins::dubins_shortest_path(x0, y0, th0, xf, yf, alpha, Kmax, pidx);
        collision = curveCollision(curve, obstacle_list);

        if (!collision) {
            break;
        }
    }

    if (!collision) {
        // add curve to final path
        multipoint_dubins_path.push_back(curve);
        thf = alpha;

        ///////////////////////
        drawDubinsArc(curve.a1);
        drawDubinsArc(curve.a2);
        drawDubinsArc(curve.a3);

        return true;
    } else {
        int k = (i+j)/2;

        if (j-i > 1) {
            bool r1 = recursiveMDP(i, k, th0, thf, short_path, multipoint_dubins_path, Kmax, pidx, obstacle_list);
            bool r2 = recursiveMDP(k, j, thf, thf, short_path, multipoint_dubins_path, Kmax, pidx, obstacle_list);

            return r1 && r2;
        } else {
            return false;
        }
    }
}
*/

bool MDP(double th0, double thf, vector<Point>& short_path,
         vector<dubins::Curve>& multipoint_dubins_path, double& Kmax, int& pidx,
         const vector<Polygon>& obstacle_list) {

    bool collision = false;
    dubins::Curve curve = dubins::Curve(0,0,0,0,0,0,0,0,0);
    dubins::Curve best_curve = dubins::Curve(0,0,0,0,0,0,0,0,0);
    double shortest_L = 10000;
    double x0, y0, xf, yf;
    double alpha = th0;

    for (int i = 0; i < short_path.size()-1; i++) {
        x0 = short_path[i].x;
        y0 = short_path[i].y;
        xf = short_path[i+1].x;
        yf = short_path[i+1].y;

        if (i == short_path.size()-1) {
            curve = dubins::dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax, pidx);
            collision = curveCollision(curve, obstacle_list);
        } else {
            for (int j = 0; j < 16; j++) {
                alpha += 6.28/16*j; //radians
                curve = dubins::dubins_shortest_path(x0, y0, th0, xf, yf, alpha, Kmax, pidx);
                collision = curveCollision(curve, obstacle_list);

                if ((!collision) && (curve.L < shortest_L)) {
                    shortest_L = curve.L;
                    best_curve = curve;
                }
            }
        }

        if (!collision) {
            multipoint_dubins_path.push_back(best_curve);
            drawDubinsArc(curve.a1);
            drawDubinsArc(curve.a2);
            drawDubinsArc(curve.a3);
            th0 = alpha;
        } else {
            cout << "ERROR: COLLISION" << endl;
            return false;
        }
    }
    return true;
}

bool pathSmoothing(int start_index, int finish_index, vector<Point> vertices,
                   vector<Polygon> obstacle_list, vector<Point>& short_path) {
    // c++;
    // cout << to_string(c) << " ";
    // if (c > 500) {
    //  return false;
    // }

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


bool planPath(const Polygon& borders, const vector<Polygon>& obstacle_list,
              const vector<pair<int,Polygon>>& victim_list, const Polygon& gate,
              const float x, const float y, const float theta, Path& path,
              const string& config_folder) {
    // #define DUBINS_DEBUG

    #ifdef PLAN_DEBUG
        printf("--------PLANNING WAS CALLED--------\n");
        fflush(stdout);
    #endif

    double xf, yf, thf;                   // Endpoint
    center_gate(gate,borders,xf,yf,thf);  // Endpoint computation
    double Kmax = 10.0;                   // Maximum curvature
    double path_res = 0.01;               // Path resolution (sampling)

    string vcd_dir = config_folder + "/../src/path-planning";

    ofstream output(vcd_dir + "/i.txt");

    if (!output.is_open()) {
        throw runtime_error("Cannot write file: " + vcd_dir + "/i.txt");
    }

    //write borders on first line
    for (int i = 0; i < borders.size(); i++) {
        if (i < borders.size()-1) {
            output << "(" << int(borders[i].x*scale) << "," << int(borders[i].y*scale) << "),";
        } else {
            output << "(" << int(borders[i].x*scale) << "," << int(borders[i].y*scale) << ")" << endl;
        }
    }

    //write vertices on second line
    int pt_x, pt_y;

    //add each obstacle in a separate line
    for (int i = 0; i < obstacle_list.size(); i++) {
        //add each vertex in (x,y) format
        for (int j = 0; j < obstacle_list[i].size(); j++) {
            pt_x = int(obstacle_list[i][j].x*scale);
            pt_y = int(obstacle_list[i][j].y*scale);

            if (j < obstacle_list[i].size()-1) {
                output << "(" << pt_x << ", " << pt_y << "), ";
            } else {
                output << "(" << pt_x << ", " << pt_y << ")" << endl;
            }
        }
    }

    //write source and destination on third line
    output << "(" << int(x*scale) << "," << int(y*scale) << "),(" << int(xf*scale) << "," << int(yf*scale) << ")";

    output.close();

    //writes in output.txt
    //first line is centers of cells and midpoints sof vertical lines
    //second line is path of ids of cells from the first line
    //string cmd = "python " + vcd_dir + "/main.py -in " + vcd_dir + "/i.txt -out " + vcd_dir + "/output.txt -algo rrt";
    string cmd = "python " + vcd_dir + "/rrt.py -in " + vcd_dir + "/i.txt -out " + vcd_dir + "/output.txt";
    char str[cmd.size()+1];
    strcpy(str, cmd.c_str());
    system(str);

    //read vertices and path from output.txt
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
                        vertices.push_back(Point(v1/scale,v2/scale));
                    }
                    count++;
                }
            }
        }
    }

    if (path_not_found) {
        throw runtime_error("Path not found!");
    }

    int pidx; // curve index
    //dubins::Curve curve = dubins::dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax, pidx); //TODO: keep or remove? decide

    cout << "Total steps in path: " << to_string(vertices.size()) << endl;

    vector<Point> short_path;
    short_path.push_back(vertices[0]);
    bool path_smoothed = pathSmoothing(0, vertices.size()-1, vertices, obstacle_list, short_path);
    // TODO: repeat smoothing until length of path does not get smaller

    cout << "SHORT PATH length: " << to_string(short_path.size()) << endl;

    for (int i = 0; i < borders.size(); i++)
        cv::line(img, cv::Point(borders[i].x*scale, borders[i].y*scale), cv::Point(borders[i+1].x*scale,borders[i+1].y*scale), cv::Scalar(0,235,0),2); // draw the line

/*      //TODO: keep or remove? decide
    for (int i = 0; i < vertices.size(); i++) {
        cv::circle(img, cv::Point(vertices[i].x*scale, vertices[i].y*scale), 2, cv::Scalar(255,0,0),CV_FILLED);
        cout << to_string(i) << ": " << to_string(vertices[i].x) << "," << vertices[i].y << endl;
    }

    for (int i = 0; i < short_path.size(); i++) {
        cv::circle(img, cv::Point(short_path[i].x*scale, short_path[i].y*scale), 2, cv::Scalar(255,235,0),CV_FILLED);
        cout << to_string(short_path[i].x) << "," << short_path[i].y << endl;
    }
    cv::flip(img, img, 0);
    cv::imshow("curves",img); ///////////////////
    cv::waitKey(0);
    */

    vector<dubins::Curve> multipoint_dubins_path;
    // Startpoint
    double x0 = short_path[0].x;
    double y0 = short_path[0].y;
    double th0 = theta;

    // add borders for collision check
    vector<Polygon> boundaries = obstacle_list;
    boundaries.push_back(borders);
    //bool path_planned = recursiveMDP(0, total_steps-1, th0, thf, short_path, multipoint_dubins_path, Kmax, pidx, boundaries);
    bool path_planned = MDP(th0, thf, short_path, multipoint_dubins_path, Kmax, pidx, boundaries);

    if (path_planned) {
        cout << "Path planned successfully! " << endl;
    } else {
        throw runtime_error("Could NOT plan path!");
    }

    vector<Pose> points;

    for (int i = 0; i < multipoint_dubins_path.size(); i++) {
        // Sample the curve with resolution @path_res
        vector<dubins::Position> res = multipoint_dubins_path[i].discretizeSingleCurve(path_res);

        // Path conversion into compatible output representation
        for (dubins::Position p : res) {
            Pose pose(p.s,p.x,p.y,p.th,p.k);
            points.push_back(pose);
        }
    }

    //Set output
    path.setPoints(points);

    /* //TODO: keep or remove? decide
    #ifdef DUBINS_DEBUG
        printf("L: %f\n", curve.L);
        printf("a1: %f [%f,%f,%f]>>[%f,%f,%f]\n", curve.a1.L,
          curve.a1.x0,
          curve.a1.y0,
          curve.a1.th0,
          curve.a1.xf,
          curve.a1.y0,
          curve.a1.thf);
        printf("a2: %f [%f,%f,%f]>>[%f,%f,%f]\n", curve.a2.L,
          curve.a2.x0,
          curve.a2.y0,
          curve.a2.th0,
          curve.a2.xf,
          curve.a2.y0,
          curve.a2.thf);
        printf("a3: %f [%f,%f,%f]>>[%f,%f,%f]\n", curve.a3.L,
          curve.a3.x0,
          curve.a3.y0,
          curve.a3.th0,
          curve.a3.xf,
          curve.a3.y0,
          curve.a3.thf);
    #endif

    ////   Sample the curve with resolution @path_res
    vector<dubins::Position> res = curve.discretizeSingleCurve(path_res);
    ////   Path conversion into compatible output representation
    vector<Pose> points;
    for (dubins::Position p : res) {
        Pose pose(p.s,p.x,p.y,p.th,p.k);
        points.push_back(pose);
    }
    path.setPoints(points); //Set output

    */

    return true;
  }
}
