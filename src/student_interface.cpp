/** \file student_interface.cpp
 * @brief Main project library
 * Library containing main functions for the functiong of the robot
*/
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

#include "clipper_helper.hpp"
#include "corner_detection.hpp"
#include "polygon_utils.hpp"

#define AUTO_CORNER_DETECTION true  ///< Use Automatic corner detection
#define COLOR_TUNING_WIZARD false    ///< Use color tuning panel

// -------------------------------- DEBUG FLAGS --------------------------------
// - Configuration Debug flags - //
// #define DEBUG_COLOR_RANGE // Prints info about HSV ranges used for colors
// #define DEBUG_COLOR_CONFIG

// - Calibration Debug flags - //
// #define DEBUG_EXTRINSIC_CALIB
// #define DEBUG_CORNER_AUTODETECT
// #define DEBUG_FINDPLANETRANSFORM

// - Image Analysis Debug flags - //
// #define DEBUG_FINDOBSTACLES
// #define DEBUG_FINDGATE
// #define DEBUG_FINDVICTIMS
// #define DEBUG_FINDROBOT

// - Planning Debug flags - //
#define DEBUG_PLANPATH            ///< generic info about the whole planner
// #define DEBUG_COMPUTEARRIVAL
// #define DEBUG_PLANPATH_SEGMENTS   ///< show images with the goals of every planned segment
// #define DEBUG_RRT                 ///< inner planning algorithm
// #define DEBUG_PATH_SMOOTHING      ///< path smoothing pipeline
 #define DEBUG_DRAWCURVE             ///< dubins path plotting
#define DEBUG_SCORES              ///< track times and scores of victims to collect
// #define DEBUG_COLLISION           ///< plot for collision detection

using namespace std;

using matrix = std::vector<std::vector<float>>;

enum class Mission { mission1, mission2 }; ///< Planning tasks available.

// --------------------------------- GLOBAL VARIABLES ---------------------------------
Mission mission = Mission::mission2;   ///< Planning task chosen.

float bonus = 0.08f;                  ///< Time bonus for each victim collected.

int angle_increment = 1;

// --------------------------------- CONSTANTS ---------------------------------
const string COLOR_CONFIG_FILE = "/color_parameters.config";

const int pythonUpscale = 1000; ///< Scale factor for planner script.
                                ///< Scale factor used to convert parameters to
                                ///< an int represetation for the planning
                                ///> library
const double debugImagesScale = 512.82; ///< Arbitrary scale factor used for
                                        ///< displaying debug images
const float SAFETY_INFLATE_AMOUNT = 0.04;    ///< Obstacles inflation amount.
                                             ///< obstacles are slightly inflated
                                             ///< by this amount to account for
                                             ///< approximation errors in the
                                             ///< computation of collisions
                                             ///< (in the RRT script)
                                             ///< (Note: value in meters)
const bool DO_CUT_GATE_SLOT = true;   ///< Cut a slot for the gate in the
                                      ///< safe arena border polygon.
                                      ///< if False, the robot only
                                      ///< arrives near the gate (not
                                      ///< inside) to avoid the possible
                                      ///< virtual collision with the arena
                                      ///< border behind the gate
const float SAFETY_GATE_INFLATE_AMOUNT = 0.01; ///<< Gate polygon inflate amount

const float ROBOT_RADIUS = 0.1491;  ///< Robot radius for obstacles and
                                    ///< borders inflation.
                                    ///< Computed as the maximum
                                    ///< distance between the wheels
                                    ///< center and the robot footprint
                                    ///< borders.

const unsigned short NUMBER_OF_MP_ANGLES = 4;   ///< Number of possible planning
                                                ///< angles.
                                                ///< Number of angles used
                                                ///< normally to plan the
                                                ///< multipoint curve.

const unsigned short MP_IT_LIMIT = 10;  ///< Iteration limit for
                                        ///< multipoint Dubins curve
                                        ///< path planning attempts.
                                                
constexpr bool USE_ANGLE_HEURISTIC = false;///< Enable heuristic on angle
                                           ///< computation.
                                           ///< Setting this to true improves the
                                           ///< way that free angles are chosen
                                           ///< when planning
//Planning
const double K_MAX = 10.0;            ///< Maximum robot curvature.
const double PATH_RESOLUTION = 0.01;  ///< Path resolution (sampling).

const float ROBOT_SPEED = 0.1f;       ///< Robot speed: 0.1 m/s.

//! Main namespace containing student interface methods
namespace student {

/** Color bounds configuration for victims, robot and obstacles. */
struct Color_config {
    tuple<int,int,int> victims_lowbound; ///< Color lower-bound for victims.
    tuple<int,int,int> victims_highbound; ///< Color higher-bound for victims.
    tuple<int,int,int> robot_lowbound; ///< Color lower-bound for robot.
    tuple<int,int,int> robot_highbound; ///< Color higher-bound for robot.
    tuple<int,int,int> obstacle_lowbound1; ///< Color first lower-bound for obstacles.
    tuple<int,int,int> obstacle_highbound1; ///< Color first higher-bound for obstacles.
    tuple<int,int,int> obstacle_lowbound2; ///< Color second lower-bound for obstacles.
    tuple<int,int,int> obstacle_highbound2; ///< Color second higher-bound for obstacles.
};

//-------------------DRAWING DUBINS CURVES-------------------
/** Image used for debug drawing. */
cv::Mat dcImg = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255,255,255));

/** Loads images from the file system.
 * @param img_out Output image.
 * @param config_folder Configuration folder path.
 * @param initialized True if images were found.
 * @param img_list  List of images to load.
 * @param idx Index of the current image.
 * @param function_call_counter Iterations count.
 * @param freeze_img_n_step Hold the current image for this number of iterations.
 * @param current_img Store the image for a period, avoid to load it from file every time.
*/
void loadImage(cv::Mat& img_out, const string& config_folder) {
    static bool initialized = false;
    static vector<cv::String> img_list;
    static size_t idx = 0;
    static size_t function_call_counter = 0;
    const static size_t freeze_img_n_step = 30;
    static cv::Mat current_img;

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

/** Saves an image when S key is pressed.
 * @param img_in Input image.
 * @param topic Image topic name.
 * @param config_folder Configuration folder path.
*/
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
        int state = system(command.c_str());  //use bash command
        if (state != 0)
            throw std::logic_error("Python script returned bad value");
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

/** Loads color bounds configuration from file.
 * @param config_folder Configuration folder path.
*/
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

/** Open color tuning panel.
 * Open a series of panels to tune the color threshold for better detection
 * @param image Reference image for color tuning.
 * @param config_folder Configuration folder path.
*/
void tune_color_parameters(const cv::Mat &image, const string& config_folder) {
    // Set destination file
    string file_path = config_folder;
    file_path += "/";
    file_path += COLOR_CONFIG_FILE;
    // Call routine in panel library
    hsvpanel::show_panel(image,file_path);
    cout << "tuned" << endl;
}

/** Performs extrinsic calibration.
 * @param img_in Input image.
 * @param object_points Array of object points in the object coordinate space.
 * @param camera_matrix Input camera matrix.
 * @param rvec Output rotation vector.
 * @param tvec Output translation vector.
 * @param config_folder Configuration folder path.
 * @return True if the function is executed successfully.
*/
bool extrinsicCalib(const cv::Mat& img_in, vector<cv::Point3f> object_points,
                    const cv::Mat& camera_matrix, cv::Mat& rvec,
                    cv::Mat& tvec, const string& config_folder) {
    vector<cv::Point2f> corners;

    if (AUTO_CORNER_DETECTION)
        corners = CornerDetection::autodetect(img_in);
    else {
        corners = CornerDetection::manualSelect(img_in,config_folder);
    }

    #ifdef DEBUG_EXTRINSIC_CALIB
        cv::line(img_in, corners[0], corners[1], cv::Scalar(0,0,255));
        cv::line(img_in, corners[1], corners[2], cv::Scalar(0,0,255));
        cv::line(img_in, corners[2], corners[3], cv::Scalar(0,0,255));
        cv::line(img_in, corners[3], corners[0], cv::Scalar(0,0,255));

        cv::circle(img_in, corners[0], 20, cv::Scalar(0,0,255),4);
        cv::circle(img_in, corners[1], 20, cv::Scalar(0,255,0),4);
        cv::circle(img_in, corners[2], 20, cv::Scalar(255,0,0),4);
        cv::circle(img_in, corners[3], 20, cv::Scalar(0,0,0),4);

        cv::putText(img_in, "0", corners[0], cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2);
        cv::putText(img_in, "1", corners[1], cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2);
        cv::putText(img_in, "2", corners[2], cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2);
        cv::putText(img_in, "3", corners[3], cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2);

        // display
        cv::imshow("Selected border points", img_in);
        cv::waitKey(0);
    #endif

    cv::Mat nullmat;
    cv::solvePnP(object_points,corners, camera_matrix, nullmat, rvec, tvec);

    // Call the routine with gui to tune the color values
    if(COLOR_TUNING_WIZARD)
        tune_color_parameters(img_in, config_folder);
    return true;
}

/** Undistorts the input image.
 * @param img_in Input distorted image.
 * @param img_out Output corrected image.
 * @param cam_matrix Input camera matrix.
 * @param dist_coeffs Input vector of distortion coefficients.
*/
void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out,
                    const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs,
                    const string& config_folder) {

    cv::undistort(img_in,img_out,cam_matrix,dist_coeffs);
}

/** Calculates a perspetive transform.
 * Calculates a perspetive transform from four pairs of the corresponding points.
 * @param cam_matrix Input camera matrix.
 * @param rvec Output rotation vector.
 * @param tvec Output translation vector.
 * @param object_points_plane Array of object points.
 * @param dest_image_points_plane Coordinates of the corresponding quadrangle vertices in the destination image.
 * @param plane_transf Output perspective transform.
*/
void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec,
                        const cv::Mat& tvec,
                        const vector<cv::Point3f>& object_points_plane,
                        const vector<cv::Point2f>& dest_image_points_plane,
                        cv::Mat& plane_transf, const string& config_folder) {
    #ifdef DEBUG_FINDPLANETRANSFORM
        cout << "findPlaneTransform called" << endl;
    #endif
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
}

/** Applies a perspective transform to an image.
 * @param img_in Input image.
 * @param img_out Output unwarped image.
 * @param transf Transformation matrix.
 * @param config_folder Configuration folder path.
*/
void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf,
            const string& config_folder) {

    cv::warpPerspective(img_in, img_out, transf, img_in.size());
}

/** Find arena obstacles.
 * Finds the obstacles in the arena given the arena image and dilate to account
 * for robot dimensions.
 * Obstacle color is red.
 * @param hsv_img HSV input image.
 * @param scale Scaling factor.
 * @param obstacle_list List of output obstacle polygons.
 * @param color_config Color bounds configuration.
*/
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
    float robot_dim = ceil(ROBOT_RADIUS * scale);

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
    for (size_t i=0; i<contours.size(); ++i) {
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

/** Finds the gate in the arena.
 * Gate color is green.
 * @param hsv_img HSV input image.
 * @param scale Scaling factor.
 * @param gate Polygon that represents the gate.
 * @param color_config Color bounds configuration.
 * @return True if gate was found.
*/
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

/** Finds the victims in the arena given the arena image and detects victim number.
 * Victim color is green. Number is detected by template matching. Template matching is performed
 * by extracting the axes-aligned minimal bounding rectangle for each region of interest and comparing it
 * with the templates in four 90 degree orientations to maximize the matching score.
 * @param hsv_img HSV input image.
 * @param scale Scaling factor.
 * @param victim_list List of output victim polygons.
 * @param color_config Color bounds configuration.
 * @param config_folder Configuration folder path.
 * @return True if victims were found.
*/
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
    vector<Polygon> polygonsFound(contours.size());

    for (size_t i=0; i<contours.size(); ++i) {
        approxPolyDP(contours[i], approx_curve, 10, true);

        if (approx_curve.size() != 4) {  // ignore gate

            Polygon scaled_contour;
            for (const auto& pt: approx_curve) {
                scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
            }
            polygonsFound[i] = scaled_contour;

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
    for (int i = 0; i <= 5; ++i) {
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

    // For each green blob in the original image containing a digit
    for (size_t i=0; i < boundRect.size(); ++i) {
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
        for (size_t j = 0; j < templROIs.size(); ++j) {
            cv::Mat result;
            cv::matchTemplate(processROI, templROIs[j], result, cv::TM_CCOEFF);
            double score;
            cv::minMaxLoc(result, nullptr, &score);
            if (score > maxScore) {
                maxScore = score;
                maxIdx = floor(j/4);
            }
        }

        if (maxIdx != -1) {
            victim_list.push_back({maxIdx, polygonsFound[i]});
        }

        cout << "Best fitting template: " << ((maxIdx==-1) ? "None (rejected)" : to_string(maxIdx)) << " coordinates: (" << PUtils::baricenter(polygonsFound[i]).x << "," << PUtils::baricenter(polygonsFound[i]).y << ")" << endl;

        #ifdef DEBUG_FINDVICTIMS
            cv::waitKey(0);
        #endif
    }

    return true;
}

/** Initiates the map processing starting from an image of the arena.
 * Converts the input image in HSV space for better color detection and
 * calls the functions that detect gate, obstacles and victims.
 * @param img_in Input image.
 * @param scale Scaling factor.
 * @param obstacle_list List of output obstacle polygons.
 * @param victim_list List of output victim polygons.
 * @param gate Polygon that represents the gate.
 * @param config_folder Configuration folder path.
 * @return True if function was executed successfully.
*/
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

/** Finds the robot in the arena given the arena image.
 * @param img_in Input image.
 * @param scale Scaling factor.
 * @param triangle Triangular polygon representing the robot.
 * @param x Robot position x coordinate.
 * @param y Robot position y coordinate.
 * @param theta Robot position angle.
 * @param config_folder Configuration folder path.
 * @return True if robot was found.
*/
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
    for (size_t i = 0; i<contours.size(); ++i) {
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
        PUtils::baricenter(triangle,cx,cy);

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

        cv::Point cv_center(x*scale, y*scale); // convert back m to px
        cv::Point cv_vertex(top_vertex.x*scale, top_vertex.y*scale); // convert back m to px
        #ifdef DEBUG_FINDROBOT
            // Draw over the image
            cv::line(contours_img, cv_center, cv_vertex, cv::Scalar(0,255,0), 3);
            cv::circle(contours_img, cv_center, 5, cv::Scalar(0,0,255), -1);
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

/** Find the arrival point.
 * Returns the baricenter of the gate polygon with the correct arrival angle,
 * along with the projection of the gate center into the nearest arena border line
 *
 * @param gate Gate polygon.
 * @param borders Arena borders polygon.
 * @param cBorders Safe-corrected arena borders polygon (offsetted borders).
 * @param x Output arrival x coordinate.
 * @param y Output arrival y coordinate.
 * @param theta Output arrival angle.
 * @param xProj Output gate center projected on cBorders, x coordinate.
 * @param yProj Output gate center projected on cBorders, y coordinate.
*/
void computeArrival(const Polygon& gate, const Polygon& borders, const Polygon& cBorders, double& x,
                 double& y, double& theta, double& xProj, double& yProj) {

    assert(gate.size() == 4);
    assert(borders.size() == 4);
    #ifdef DEBUG_COMPUTEARRIVAL
        printf("---Center gate called---\n");
        printf("There are %zd points\n", gate.size());
    #endif

    ////   GATE BARICENTER
    double gc_x=0, gc_y=0;
    PUtils::baricenter(gate,gc_x,gc_y);

    ////   Find shortest gate side (between the first two)
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

    ////   Find GATE ANGLE
    double angle;
    angle = atan2(shortest_p0.y-shortest_p1.y,shortest_p0.x-shortest_p1.x);
    if (angle < 0)
        angle += 2 * CV_PI;
    // Decide if the angle is the found one or the opposite (+180°) based on the position of the gate
    double ac_x = 0, ac_y = 0;
    PUtils::baricenter(borders,ac_x,ac_y);
    if (gc_y > ac_y) {
        angle -= CV_PI;
        if (angle < 0)
            angle += 2 * CV_PI;
    }

    //
    //   Project the solution into the corrected borders
    //

    // cBorders are the restricted borders that account for robot footprint when
    // planning a line path and this projection places the arrival point inside
    // this space

    // Offset cBorders of a small safety amount to be sure that the arrival
    // doesn't lie exactly on the border
    Polygon safecBorders = ClipperHelper::offsetBorders(cBorders, -1.0 * SAFETY_GATE_INFLATE_AMOUNT);

    vector<Point> projections;
    for (size_t i = 0; i < safecBorders.size(); ++i) {
        Point pA = safecBorders[i],
              pB = safecBorders[(i+1)%safecBorders.size()];
        Point pP = PUtils::projectPointToLine(pA,pB,Point(gc_x, gc_y));
        projections.push_back(pP);
    }

    Point pointProjection = PUtils::nearestPoint(Point(gc_x,gc_y),projections);

    #ifdef DEBUG_COMPUTEARRIVAL
        cv::circle(dcImg, cv::Point(gc_x * debugImagesScale, gc_y * debugImagesScale), 5, cv::Scalar(255,0,0), -1);
        cv::circle(dcImg, cv::Point(pointProjection.x * debugImagesScale,pointProjection.y * debugImagesScale), 5, cv::Scalar(0,255,0), -1);
    #endif

    ////   Assign Output
    x = gc_x;
    y = gc_y;
    xProj = pointProjection.x;
    yProj = pointProjection.y;
    theta = angle;
}

/** Cut a slot in the safeBorders polygon for the gate.
 *
 * @param gate gate polygon
 * @param cBorders safe Corrected Borders polygon
 * @param xf xcoord of the projection of the gate center on the safe Borders
 * @param yf ycoord of the projection of the gate center on the safe Borders
*/
Polygon cutGateSlot(const Polygon& gate, const Polygon& cBorders, double xf,
                    double yf) {
    Polygon slottedBorders = cBorders;
    // Find the two corners of the side of the gate
    int aIdx = -1,bIdx = -1;
    for (size_t i = 0; i < cBorders.size(); ++i) {
        if ((abs(cBorders[i].x - xf) < (SAFETY_GATE_INFLATE_AMOUNT*1.1)) || \
            (abs(cBorders[i].y - yf) < (SAFETY_GATE_INFLATE_AMOUNT*1.1))) {
            if(aIdx == -1) {
                aIdx = i;
            } else if(bIdx == -1) {
                bIdx = i;
            }
        }
    }
    assert((aIdx != -1) && (bIdx != -1));

    //find the outer points of the gate, to project into the borders
    int gAidx = -1, gBidx = -1;
    float maxDistance = std::numeric_limits<float>::min();
    for (size_t i = 0; i < gate.size(); ++i) {
        float distance = pow(gate[i].x - xf,2) + pow(gate[i].y - yf,2);
        if (distance > maxDistance) {
            maxDistance = distance;
            gAidx = i;
        }
    }
    maxDistance = std::numeric_limits<float>::min();
    for (size_t i = 0; i < gate.size(); ++i) {
        if(i != (size_t)gAidx) {
            float distance = pow(gate[i].x - xf,2) + pow(gate[i].y - yf,2);
            if (distance > maxDistance) {
                maxDistance = distance;
                gBidx = i;
            }
        }
    }

    assert((gAidx != -1) && (gBidx != -1));

    //Project points into the cBorders
    Point gAproj = PUtils::projectPointToLine(cBorders[aIdx],cBorders[bIdx],gate[gAidx]);
    Point gBproj = PUtils::projectPointToLine(cBorders[aIdx],cBorders[bIdx],gate[gBidx]);

    // Create the polygon to add to the arena safe borders
    Polygon gatePoly = {gAproj, gBproj, gate[gAidx], gate[gBidx]};
    PUtils::sortPolygonPoints(gatePoly);

    // Merge gate and borders polygon
    slottedBorders = ClipperHelper::mergePolygons(cBorders,gatePoly);

    #ifdef DEBUG_CUTSLOT
        int counter = 0;
        for (Point bp : slottedBorders) {
            counter++;
            cv::circle(dcImg, cv::Point(bp.x * debugImagesScale, bp.y * debugImagesScale), 5, cv::Scalar(0,0,255), -1);
            cv::putText(dcImg, std::to_string(counter), cv::Point(bp.x*debugImagesScale, bp.y*debugImagesScale), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,0), 2);
        }
        cv::imshow("CutSlot", dcImg);
        cv::waitKey(0);
    #endif

    return slottedBorders;
}

/** Checks if a dubins::Arc is colliding with a segment.
 * @param a Arc to check.
 * @param pA First point of the segment.
 * @param pB Second point of the segment.
 * @return True if arc and segment are intersecting.
*/
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

/** Checks if a segment is colliding with another segment.
 * @param a1 First point of the first segment.
 * @param a2 Second point of the second segment.
 * @param p1 First point of the second segment.
 * @param p2 Second point of the second segment.
 * @return True if the segments are intersecting.
*/
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

/** Approximate collision checker.
 * Approximate approach to check for collisions between dubins::Arc and segment using arc discretization.
 * @param a Arc to check.
 * @param pA First point of the segment.
 * @param pB Second point of the segment.
 * @return True if the arc and segment are intersecting.
*/
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
        for (size_t j = 0; j < d_arc.size()-1; j++)
        {
            cv::line(img, cv::Point(d_arc[j].x*debugImagesScale, d_arc[j].y*debugImagesScale),
                cv::Point(d_arc[j+1].x*debugImagesScale, d_arc[j+1].y*debugImagesScale), cv::Scalar(255,200,0),1); // draw the line
        }
    #endif

    #ifdef DEBUG_COLLISION
        cv::line(img, cv::Point(p[i].x*debugImagesScale, p[i].y*debugImagesScale),
            cv::Point(p[i+1].x*debugImagesScale, p[i+1].y*debugImagesScale), cv::Scalar(0,0,255),1); // draw the line
    #endif

    for (size_t j = 0; j < d_arc.size()-1; j++)
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

/** Checks if a dubins::Arc is colliding with a given polygon.
 * A dubins::Arc can also be a segment.
 * @param a Arc to check.
 * @param p The polygon to check.
 * @return True if the arc is intersecting the polygon.
*/
bool isCollidingWithPolygon(dubins::Arc& a, Polygon p) {
    float k = a.k;

    // add first vertex again to check segment between first and last
    p.push_back(p[0]);

    // if k=0 a is a segment, else it is an arc

    // check each segment in polygon p
    for (size_t i = 0; i < p.size()-1; i++) {
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

/** Checks if a dubins::Curve is colliding with any obstacle.
 * The function performs three checks, one for each dubins::Arc component of the dubins::Curve.
 * @param curve Curve to check.
 * @param obstacle_list The list of obstacle polygons to check.
 * @return True if the curve is intersecting any obstacle.
*/
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

/** Draws a dubins::Arc on a debug image.
 * @param da Arc to draw.
*/
void drawDubinsArc(dubins::Arc& da) {

    cv::Point2f startf = cv::Point2f(da.x0*debugImagesScale, da.y0*debugImagesScale);
    cv::Point2f finishf = cv::Point2f(da.xf*debugImagesScale, da.yf*debugImagesScale);

    if (da.k == 0) {
        cv::line(dcImg, cv::Point(startf.x,startf.y), cv::Point(finishf.x,finishf.y), cv::Scalar(255,200,0),1); // draw the line
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
                cv::line(dcImg, startSegment, finishSegment, cv::Scalar(255,200,0),1); // draw the line
            }
        } else { // counter clockwise segment drawing
            for (unsigned int i = 0; 0.01 < ((dubins::mod2pi(thetastart - passo*i) - thetafinish)*(dubins::mod2pi(thetastart - passo*i) - thetafinish)); i++) {
                cv::Point startSegment = cv::Point(cos(thetastart - passo*i  )*radius + centerf.x,+ sin(thetastart - passo*i  )*radius + centerf.y);
                cv::Point finishSegment = cv::Point(cos(thetastart - passo*(i+1)  )*radius + centerf.x, sin(thetastart - passo*(i+1))*radius + centerf.y);
                cv::line(dcImg, startSegment, finishSegment, cv::Scalar(255,200,0),1); // draw the line
           }
        }

        cv::circle(dcImg,start,2,cv::Scalar(0,0,0),3);
        cv::circle(dcImg,finish,2,cv::Scalar(0,0,0),3);
    }
}

/** Finds the shortest multi-point dubins curve for the given path.
 * The recursive function has two base cases: two-points dubins::Curve and three-points dubins::Curve.
 * The recursion step is called on N-points dubins::Curve. Free angles are chosen among a number of
 * test angles.
 * @param path List of points in the path.
 * @param startIdx Recursion start index.
 * @param arriveIdx Recursion arrival index.
 * @param startAngle Starting angle.
 * @param arriveAngle Arrival angle.
 * @param returnedLength Output length of the shortest path.
 * @param obstacle_list List of obstacle polygons.
 * @param num_angles Number of angles to test.
 * @return A pair with true if the path does not collide and the shortest multi-point dubins::Curve.
*/
std::pair<bool,std::vector<dubins::Curve>> MDP(const std::vector<Point> &path,
                                               unsigned int startIdx, unsigned int arriveIdx,
                                               double startAngle, double arriveAngle,
                                               double& returnedLength,
                                               const vector<Polygon>& obstacle_list,
                                               const unsigned short num_angles) {
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
        // Compute the curve for segment A and check for collisions
        int pidx = 0;
        double x1 = path[startIdx].x;
        double y1 = path[startIdx].y;
        double theta1 = startAngle;
        double x2 = path[arriveIdx].x;
        double y2 = path[arriveIdx].y;
        double theta2 = arriveAngle;
        dubins::Curve curve = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, K_MAX, pidx);
        bool isColliding = isCurveColliding(curve, obstacle_list);

        returnedLength = isColliding ? std::numeric_limits<double>::max() : curve.L;
        std::vector<dubins::Curve> multipointPath(path.size()-1);    // segments are one less than the num of points
        if (!isColliding)
            multipointPath.at(startIdx) = curve; //ERRORE?

        return std::make_pair(!isColliding,multipointPath);  // Return true if the path does not collide
    }

    // RECURSION BASE CASE 2: Called on three points it checks for num_angles for the middle point
    //            o---o---o
    // nodes:     1   2   3
    // segments:    A   B
    if (arriveIdx-startIdx == 2) {
        std::pair<dubins::Curve,dubins::Curve> bestCurves;
        double bestLength =  std::numeric_limits<double>::max();
        bool allAnglesResultInCollision = true; // If no angle choice provides a non-colliding path, the call must fail

        for (int i = 0; i < num_angles; ++i) {
            double alpha_middlepoint = (2*M_PI)/num_angles * i;
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
            dubins::Curve curveA = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, K_MAX, pidx);
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
            dubins::Curve curveB = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, K_MAX, pidx);
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
    // from node n-1 are free and they are chosen from a pool of num_angles
    // according to which ones correspond to the shortest non-colliding path.
    // NOTE: the arrival angle for node 2 is called alpha_first and the exit
    // angle from node n-1 is called alpha_second

    std::pair<dubins::Curve,dubins::Curve> bestCurves;  // best dubins curves yet
    double bestLength = std::numeric_limits<double>::max();
    std::vector<dubins::Curve> bestRecursivePath;
    bool allAnglesResultInCollision = true; // If no angle choice provides a non-colliding path, the call must fail

    for (int i = 0; i < num_angles; ++i) {
        for (int j = 0; j < num_angles; ++j) {
            double alpha_first = (2*M_PI)/num_angles * i;
            double alpha_second = (2*M_PI)/num_angles * j;

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
            dubins::Curve curveA = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, K_MAX, pidx);
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
            dubins::Curve curveB = dubins::dubins_shortest_path(x1, y1, theta1, x2, y2, theta2, K_MAX, pidx);
            bool isBColliding = isCurveColliding(curveB, obstacle_list);

            double recursivelyReturnedLength = -1;

            bool result = false;
            std::vector<dubins::Curve> recursiveReturnedPath;
            if ((!isAColliding) && (!isBColliding)) {
                std::pair<bool,std::vector<dubins::Curve>> tuple;
                tuple = MDP(path, startIdx+1,arriveIdx-1,
                            alpha_first,alpha_second,
                            recursivelyReturnedLength,
                            obstacle_list,
                            num_angles);
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

/** Checks if a path is colliding with any obstacle.
 * @param vertices List of points in the path.
 * @param obstacle_list List of obstacle polygons.
 * @return True if the path is colliding with any obstacle.
*/
bool isPathColliding(vector<Point> vertices, vector<Polygon> obstacle_list) {
    for (size_t i = 1; i < vertices.size(); ++i) {
        double x0, y0, xf, yf;
        x0 = vertices[i-1].x;
        y0 = vertices[i-1].y;
        xf = vertices[i].x;
        yf = vertices[i].y;

        for (Polygon p : obstacle_list) {
            for (size_t k = 0; k < p.size(); k++) {
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

/** Tries to reduce the number of points in a path recursively.
 * Splits the path into two and performs recursion.
 * @param start_index Index of the starting point.
 * @param finish_index Index of the arrival point.
 * @param vertices Path to smooth.
 * @param obstacle_list List of obstacle polygons.
 * @param short_path Output smoothed path.
 * @return True if a path with fewer points was found.
*/
bool pathSmoothing(int start_index, int finish_index, vector<Point> vertices,
                   vector<Polygon> obstacle_list, vector<Point>& short_path) {
    bool collision = false;
    double x0, y0, xf, yf;

    x0 = vertices[start_index].x;
    y0 = vertices[start_index].y;

    xf = vertices[finish_index].x;
    yf = vertices[finish_index].y;

    for (Polygon p : obstacle_list) {
        for (size_t k = 0; k < p.size(); k++) {
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

/** Plans the path with RRT.
 * Prepares a file with input data: starting point, arrival point, borders and obstacles.
 * The RRT library will output another file with the coordinates of the points in the path found.
 * @param borders Borders of the arena.
 * @param obstacle_list List of obstacle polygons.
 * @param x0 Starting point x coordinate.
 * @param y0 Starting point y coordinate.
 * @param xf Arrival point x coordinate.
 * @param yf Arrival point y coordinate.
 * @param config_folder  Configuration folder path.
 * @return The planned path.
*/
vector<Point> RRTplanner(const Polygon& borders, const vector<Polygon>& obstacle_list,
                  const float x0, const float y0, const float xf, const float yf,
                  const string& config_folder) {

    //
    // write the problem parameters to a file that will be fed to a planning lib
    //

    const string plan_script_lib = config_folder + "/../src/path-planning";
    ofstream output(plan_script_lib + "/i.txt");
    if (!output.is_open()) {
        throw runtime_error("Cannot write file: " + plan_script_lib + "/i.txt");
    }

    #ifdef DEBUG_RRT
        printf("To avoid approximation errors, obstacles are inflated by %f meters (%f cm)\n",SAFETY_INFLATE_AMOUNT,SAFETY_INFLATE_AMOUNT*100);
    #endif

    vector<Polygon> inflated_obstacle_list = ClipperHelper::inflatePolygons(obstacle_list,SAFETY_INFLATE_AMOUNT);

    #ifdef DEBUG_RRT
        printf("Writing problem parameters to file\n");
        printf("Measures are upscaled by a scale factor of: %d\n",pythonUpscale);
    #endif

    // write borders on the first line
    for (size_t i = 0; i < borders.size(); i++) {
        if (i < borders.size()-1) {
            output << "(" << int(borders[i].x*pythonUpscale) << "," << int(borders[i].y * pythonUpscale) << "),";
        } else {
            output << "(" << int(borders[i].x*pythonUpscale) << "," << int(borders[i].y * pythonUpscale) << ")" << endl;
        }
    }

    //  write vertices on the second line
    for (size_t i = 0; i < inflated_obstacle_list.size(); i++) {
        //add each vertex in (x,y) format
        for (size_t j = 0; j < inflated_obstacle_list[i].size(); j++) {
            int pt_x = int(inflated_obstacle_list[i][j].x * pythonUpscale);
            int pt_y = int(inflated_obstacle_list[i][j].y * pythonUpscale);

            if (j < inflated_obstacle_list[i].size() - 1) {
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
    string cmd = "python " + plan_script_lib + "/rrt.py -in " + plan_script_lib + "/i.txt -out " + plan_script_lib + "/output.txt";
    // call library script
    int state = system(cmd.c_str());
    if (state != 0)
        throw std::logic_error("Python script returned bad value");

    //
    // Read the resulting path
    //

    // read vertices and path from output.txt
    ifstream input(plan_script_lib + "/output.txt");
    vector<Point> vertices;

    bool path_not_found = false;
    if (input.is_open()) {
        string line;
        while (getline(input,line)) {
            input >> line;
            istringstream ss(line);
            string token;

            float v1 = 0.0, v2 = 0.0;
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

/** Tries to reduce the number of points in a path by combinaning different techniques.
 * The function performs recursive smoothing iteratively until
 * no change is observed. An additional step is performed on the points of the path
 * in reversed order to achieve further smoothing.
 * @param path Input path to smooth.
 * @param obstacle_list List of obstacle polygons.
 * @return The smoothed path.
*/
vector<Point> completeSmoothing(const vector<Point>& path, const vector<Polygon>& obstacle_list) {
    vector<Point> smoothedPath;
    smoothedPath.push_back(path[0]);
    bool is_path_smoothed = pathSmoothing(0, path.size()-1, path, obstacle_list, smoothedPath);

    if (is_path_smoothed) {
        #ifdef DEBUG_PATH_SMOOTHING
            cout << "\t>Path shortened ONCE  (size: " << smoothedPath.size() << ")" << endl;
        #endif
        bool additional_shortening = true;

        while (additional_shortening) {
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
        }

        assert(PUtils::pointsEquals(smoothedPath.front(),path.front()));
        assert(PUtils::pointsEquals(smoothedPath.back(),path.back()));

        // additional iteration on reversed path
        vector<Point> shorter_path;
        std::reverse(smoothedPath.begin(), smoothedPath.end());
        shorter_path.push_back(path[path.size()-1]);
        bool reverse_smoothing = pathSmoothing(0, smoothedPath.size()-1, smoothedPath, obstacle_list, shorter_path);
        if (reverse_smoothing && (shorter_path.size() < smoothedPath.size())) {
            #ifdef DEBUG_PATH_SMOOTHING
                cout << "\t>Path shortened AGAIN (size: " << smoothedPath.size() << "->" << shorter_path.size() << ")" << endl;
            #endif
            smoothedPath = shorter_path;
        }
        std::reverse(smoothedPath.begin(), smoothedPath.end());

        assert(PUtils::pointsEquals(smoothedPath.front(),path.front()));
        assert(PUtils::pointsEquals(smoothedPath.back(),path.back()));
        return smoothedPath;

    }else{
        #ifdef DEBUG_PATH_SMOOTHING
            cout << "Path smoothing FAILED! Collision detected" << endl;
        #endif
        throw logic_error("ERROR: path cannot be shortened");
    }
}

/** Draws the path on a debug image.
 * @param path The path to draw.
*/
void drawDebugPath(std::vector<Point> path) {
    for (size_t i = 0; i < path.size(); i++) {
        cv::circle(dcImg, cv::Point(path[i].x*debugImagesScale, path[i].y*debugImagesScale), 2, cv::Scalar(0,0,0),CV_FILLED);
        if (i > 0)
            cv::line(dcImg, cv::Point(path[i-1].x*debugImagesScale, path[i-1].y*debugImagesScale),
                     cv::Point(path[i].x*debugImagesScale, path[i].y*debugImagesScale), cv::Scalar(255,0,0),2);
    }
}

/** Draws borders, obstacles and victims on a debug image.
 * @param borders Borders of the arena
 * @param obstacle_list List of obstacle polygons.
 * @param victim_list List of victim polygons.
*/
void drawDebugImage(const Polygon& borders, const vector<Polygon>& obstacle_list, const vector<pair<int,Polygon>>& victim_list){
    //
    //  Draw Curves to debug errors
    //
    dcImg = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255,255,255));

    // draw borders
    for (size_t i = 0; i < borders.size(); i++) {
        int next = (i+1)%borders.size();
        cv::line(dcImg, cv::Point(borders[i].x*debugImagesScale, borders[i].y*debugImagesScale), cv::Point(borders[next].x*debugImagesScale,borders[next].y*debugImagesScale), cv::Scalar(0,0,0),3); // draw the line

        cv::putText(dcImg, std::to_string(i), cv::Point(borders[i].x*debugImagesScale,  borders[i].y*debugImagesScale), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,0), 1);
    }
    //Draw obstacles
    for (const Polygon &pol : obstacle_list) {
        for (size_t i = 1; i<pol.size(); ++i) {
            cv::line(dcImg, cv::Point(pol[i-1].x*debugImagesScale, pol[i-1].y*debugImagesScale),
                     cv::Point(pol[i].x*debugImagesScale, pol[i].y*debugImagesScale), cv::Scalar(0,0,255),2);
        }
        cv::line(dcImg, cv::Point(pol[0].x*debugImagesScale, pol[0].y*debugImagesScale),
                     cv::Point(pol[pol.size()-1].x*debugImagesScale, pol[pol.size()-1].y*debugImagesScale), cv::Scalar(0,0,255),2);
    }
    //Draw victims
    for (const pair<int, Polygon> victim : victim_list) {
        const Polygon &pol = victim.second;
        for (size_t i = 1; i<pol.size(); ++i) {
            cv::line(dcImg, cv::Point(pol[i-1].x*debugImagesScale, pol[i-1].y*debugImagesScale),
                     cv::Point(pol[i].x*debugImagesScale, pol[i].y*debugImagesScale), cv::Scalar(0,255,0),2);
        }
        cv::line(dcImg, cv::Point(pol[0].x*debugImagesScale, pol[0].y*debugImagesScale),
                     cv::Point(pol[pol.size()-1].x*debugImagesScale, pol[pol.size()-1].y*debugImagesScale), cv::Scalar(0,255,0),2);

        Point center = PUtils::baricenter(pol);
        cv::putText(dcImg, std::to_string(victim.first), cv::Point(center.x*debugImagesScale, center.y*debugImagesScale), cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,0), 2);
    }
}

/** Plans a path in which every victim is collected in the correct order.
 * Calls the planning steps for each sub-path in the following order: RRT planner, path smoothing, multi-point
 * dubins curve problem.
 * @param safeBorders safe borders without gate slot (used to prevent RRT bug)
 * @param slotBorders borders with gate slot (used for collision detection)
 * @param obstacle_list List of obstacle polygons.
 * @param victim_list List of victim polygons.
 * @param x Starting point x coordinate.
 * @param y Starting point y coordinate.
 * @param theta Starting angle.
 * @param xf Arrival point x coordinate.
 * @param yf Arrival point y coordinate.
 * @param thf Arrival angle.
 * @param config_folder Configuration folder path.
 * @return The multi-point dubins curve solution.
*/
vector<dubins::Curve> collectVictimsPath(const Polygon& safeBorders,
                                         const Polygon& slotBorders,
                                         const vector<Polygon>& obstacle_list,
                                         const vector<pair<int,Polygon>>& victim_list,
                                         float x, float y, float theta,
                                         float xf, float yf, float thf,
                                         const string& config_folder) {
    //
    // Create a vector of crucial points (start, victims, end);
    //
    vector<Point> pathObjectives;
    pathObjectives.push_back(Point(x,y));              // push initial point
    for (const pair<int,Polygon>& victim : victim_list)
        pathObjectives.push_back(PUtils::baricenter(victim.second));  //push each victim center
    pathObjectives.push_back(Point(xf,yf));            // push final point

    //
    // Call a path planner for each segment to plan
    //

    vector<Point> full_path;
    full_path.push_back(Point(x,y));
    vector<Point> short_path;
    short_path.push_back(Point(x,y));
    for (size_t i = 1; i < pathObjectives.size(); ++i) {
        #ifdef DEBUG_PLANPATH
            cout << "Planning segment " << i << "/" << (pathObjectives.size()-1) << endl;
        #endif
        //
        // PLANNING Step 1: Call RRT planner
        //
        float x1,y1,x2,y2;
        x1 = pathObjectives[i-1].x;
        y1 = pathObjectives[i-1].y;
        x2 = pathObjectives[i].x;
        y2 = pathObjectives[i].y;

        #ifdef DEBUG_PLANPATH_SEGMENTS
            cout << "Segment (" << x1 << "," << y1 << ")->(" << x2 << "," << y2 << ")" << endl;
            dcImg = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255,255,255));
            drawDebugImage(slotBorders, obstacle_list, victim_list);
            cv::Point pointA(x1*debugImagesScale,y1*debugImagesScale);
            cv::Point pointB(x2*debugImagesScale,y2*debugImagesScale);
            cv::circle(dcImg, pointA, 20, cv::Scalar(0,255,0),4);
            cv::putText(dcImg, "A", pointA, cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2);
            cv::circle(dcImg, pointB, 20, cv::Scalar(0,0,255),4);
            cv::putText(dcImg, "B", pointB, cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2);
            cv::imshow("Current goal", dcImg);
            cv::waitKey(0);
        #endif

        vector<Point> partialPath = RRTplanner(safeBorders,obstacle_list,x1,y1,x2,y2,config_folder);
        full_path.insert(full_path.end(),partialPath.begin()+1,partialPath.end());    // begin()+1 not to repeat points
        assert(!isPathColliding(partialPath, obstacle_list));  // If the rrt path collides there is an error in the python script or conversion

        //
        // PLANNING Step 2: Smoothing/Shortcutting
        //
        vector<Point> partialShortPath = completeSmoothing(partialPath,obstacle_list);
        short_path.insert(short_path.end(), partialShortPath.begin()+1, partialShortPath.end());    // begin()+1 not to repeat points
    }
    assert(PUtils::pointsEquals(short_path.front(),full_path.front()));
    assert(PUtils::pointsEquals(short_path.back(),full_path.back()));

    #ifdef DEBUG_PLANPATH
        cout << "------------------------------------------------------------" << endl;
        cout << "> Planning Step 1: planned RRT path ("<< full_path.size() <<" steps)" << endl;
        cout << "------------------------------------------------------------" << endl;
        cout << "> Planning Step 2: smoothed path ("<< short_path.size() <<" steps)" << endl;
        cout << "------------------------------------------------------------" << endl;
    #endif

    #ifdef DEBUG_DRAWCURVE
        drawDebugImage(slotBorders, obstacle_list, victim_list);
        drawDebugPath(short_path);
        cv::imshow("Curves",dcImg);
        cv::waitKey(0);
    #endif

    //
    // PLANNING Step 3: plan a Multi-point(or Multi-curve) Dubins path
    //

    vector<Polygon> boundaries = obstacle_list;
    boundaries.push_back(slotBorders); // add slotted borders for collision check

    #ifdef DEBUG_PLANPATH
        cout << "Computing Multi Point Dubins path..." << endl;
    #endif

    unsigned short numberOfMpAngles = NUMBER_OF_MP_ANGLES;
    bool path_planned = false;
    vector<dubins::Curve> multipointPath;
    unsigned short it_count = 0;

    do {
        double returnedLength = 0;  // unused here, just for recursion
        std::pair<bool,vector<dubins::Curve>> multipointResult;
        multipointResult = MDP(short_path,
                               0, short_path.size()-1,   // start and arrival indexes
                               theta, thf,                 // start and arrive angles
                               returnedLength,
                               boundaries,
                               numberOfMpAngles);
        path_planned = multipointResult.first;
        multipointPath = multipointResult.second;

        if (path_planned) {
        #ifdef DEBUG_PLANPATH
            cout << "> Planning Step 3: Multipoint dubins curve planned successfully" << endl;
            cout << "------------------------------------------------------------" << endl;
        #endif
        } else {
            if (it_count > 0){
                numberOfMpAngles = numberOfMpAngles + angle_increment;
            }

            #ifdef DEBUG_PLANPATH
                cout << "> Planning Step 3: Could not plan path, trying with " << numberOfMpAngles << " angles" << endl;
            #endif
        }

        it_count++;

    } while (!path_planned && (it_count <= MP_IT_LIMIT));

    if (!path_planned){
        cout << "Could not plan path, iteration limit reached." << endl;
    }

    #ifdef DEBUG_DRAWCURVE

        for (size_t i = 0; i < multipointPath.size(); i++)
        {
            drawDubinsArc(multipointPath[i].a1);
            drawDubinsArc(multipointPath[i].a2);
            drawDubinsArc(multipointPath[i].a3);
        }

        //cv::flip(dcImg, dcImg, 0);
        cv::imshow("Curves",dcImg);
        cv::waitKey(0);

    #endif

    return multipointPath;
}

/** Computes the length of a path of points.
 * @param path Input path points.
 * @return The legth of the path.
*/
float getPointPathLength(const vector<Point>& path){
    float length = 0;

    for (size_t i = 0; i < path.size()-1; i++)
    {
        const Point pos1 = path[i];
        const Point pos2 = path[i+1];
        length += sqrt(pow((pos1.x - pos2.x), 2) + pow((pos1.y - pos2.y), 2));
    }

    return length;
}

/** Computes the length of a path of poses.
 * @param path Input path poses.
 * @return The legth of the path.
*/
float getPosePathLength(const vector<Pose>& path){
    float length = 0;

    for (size_t i = 0; i < path.size()-1; i++)
    {
        const Pose pos1 = path[i];
        const Pose pos2 = path[i+1];
        length += sqrt(pow((pos1.x - pos2.x), 2) + pow((pos1.y - pos2.y), 2));
    }

    return length;
}

/** Computes the length of a multi-point dubins curve.
 * @param multipointPath Input dubins curve.
 * @return The legth of the multi-point dubins curve.
*/
float getPathLength(const vector<dubins::Curve>& multipointPath){
    float length = 0;
    for (const dubins::Curve& curve : multipointPath)
        length += curve.L;
    return length;
}

/** Custom sorting function for pairs of indexed distances.
 * @param p1 First <index,distance> pair.
 * @param p2 Second <index,distance> pair.
 * @return True if the first distance value is smaller.
*/
bool sorByDistance(const pair<int,float>& p1, const pair<int,float>& p2){
    return (p1.second < p2.second);
}

/** Plans a path that maximizes the time-score of the mission.
 * For each victim collected a time-bonus is granted. At each step, the greedy function picks the victim
 * that better improves the final score. To avoid robot loops and improve the search, the victims to test are ordered by
 * distance from the starting point.
 * @param safeBorders safe borders without gate slot (used to prevent RRT bug)
 * @param slotBorders borders with gate slot (used for collision detection)
 * @param obstacle_list List of obstacle polygons.
 * @param victim_list List of victim polygons.
 * @param x Starting point x coordinate.
 * @param y Starting point y coordinate.
 * @param theta Starting angle.
 * @param xf Arrival point x coordinate.
 * @param yf Arrival point y coordinate.
 * @param thf Arrival angle.
 * @param config_folder Configuration folder path.
 * @return The multi-point dubins curve solution.
*/
vector<dubins::Curve> bestScoreGreedy(const Polygon& safeBorders,
                                      const Polygon& slotBorders,
                                      const vector<Polygon>& obstacle_list,
                                      const vector<pair<int,Polygon>>& victim_list,
                                      float x, float y, float theta,
                                      float xf, float yf, float thf,
                                      const string& config_folder) {

    vector<dubins::Curve> multipointPath;
    float best_partial_time;    // current best time score

    // compute victim distance from start
    vector<float> distances;

    for (size_t i = 0; i < victim_list.size(); i++)
    {
        vector<Point> path = RRTplanner(safeBorders,obstacle_list,x,y,PUtils::baricenter(victim_list[i].second).x,PUtils::baricenter(victim_list[i].second).y,config_folder);
        assert(!isPathColliding(path, obstacle_list));  // If the rrt path collides there is an error in the python script or conversion

        float length = getPointPathLength(path);

        distances.push_back(length);
    }

    // no victim path
    vector<bool> collected;

    for (size_t i = 0; i < victim_list.size(); i++)
        collected.push_back(false);

    vector<pair<int,Polygon>> empty_victims_vector;

    #ifdef DEBUG_SCORES
        cout << "Choosing victims for best scoring path." << endl;
    #endif

    multipointPath = collectVictimsPath(safeBorders, slotBorders, obstacle_list, empty_victims_vector, x, y, theta, xf, yf, thf, config_folder);

    float length = getPathLength(multipointPath);

    if (length > 0){
        best_partial_time = length / ROBOT_SPEED;

        #ifdef DEBUG_SCORES
            cout << "No victim. Time-score: " << (length / ROBOT_SPEED) << endl;
        #endif
    } else {
        best_partial_time = 100000;
    }

    vector<pair<int,Polygon>> victims_to_collect;
    vector<pair<int,float>> ordered_distances, temp_ordered_distances;
    int victim_idx, bonus_multiplier = 1;

    // repeat until no more victim is worth collecting or every victim is collected
    do{
        victim_idx = -1;

        for (size_t j = 0; j < victim_list.size(); j++)
        {
            if (!collected[j]) {
                vector<dubins::Curve> current_path;

                vector<pair<int,Polygon>> temp_victim_list;
                temp_ordered_distances = ordered_distances;

                temp_ordered_distances.push_back(make_pair(j,distances[j]));

                sort(temp_ordered_distances.begin(), temp_ordered_distances.end(), sorByDistance);

                for (size_t i = 0; i < temp_ordered_distances.size(); i++)
                {
                    temp_victim_list.push_back(victim_list[temp_ordered_distances[i].first]);
                }

                #ifdef DEBUG_SCORES
                        cout << "Testing with victim " << victim_list[j].first << endl;
                #endif

                current_path = collectVictimsPath(safeBorders, slotBorders, obstacle_list, temp_victim_list, x, y, theta, xf, yf, thf, config_folder);

                length = getPathLength(current_path);

                if (length > 0){
                    #ifdef DEBUG_SCORES
                        cout << "Score after collecting victim " << victim_list[j].first << ". Time: " << (length / ROBOT_SPEED);
                    #endif

                    float current_partial_time = (length / ROBOT_SPEED) - (bonus * bonus_multiplier);

                    #ifdef DEBUG_SCORES
                        cout << ", time-score: " << current_partial_time << endl;
                    #endif

                    // update score and path
                    if (current_partial_time < best_partial_time) {
                        best_partial_time = current_partial_time;
                        multipointPath = current_path;
                        victim_idx = j;
                    }
                }
            }
        }

        if (victim_idx > -1) {
            victims_to_collect.push_back(victim_list[victim_idx]);
            ordered_distances.push_back(make_pair(victim_idx, distances[victim_idx]));
            collected[victim_idx] = true;
            bonus_multiplier++;

            #ifdef DEBUG_SCORES
                cout << "Will collect victim " << victim_list[victim_idx].first << endl;
            #endif
        }
    } while(victim_idx > -1);
    return multipointPath;
}

Path savedPath;

/** Plans a path according to the mission selected.
 * @param borders Borders of the arena
 * @param obstacle_list List of obstacle polygons.
 * @param victim_list List of victim polygons.
 * @param gate Gate polygon.
 * @param x Starting point x coordinate.
 * @param y Starting point y coordinate.
 * @param theta Starting angle.
 * @param path Output planned path.
 * @param config_folder Configuration folder path.
 * @return True if the path was planned correctly.
*/
bool planPath(const Polygon& borders, const vector<Polygon>& obstacle_list,
              const vector<pair<int,Polygon>>& victim_list, const Polygon& gate,
              const float x, const float y, const float theta, Path& path,
              const string& config_folder) {

    #ifdef DEBUG_PLANPATH
        printf("--------PLANNING WAS CALLED--------\n");
        fflush(stdout);
    #endif
    
    int mission_selected;
    float bonus_selected;

    cout << "Select mission type.\n 1: collect all victims in numbered order\n 2: collect victims with highest scoring path\n";

    cin >> mission_selected;

    cout << "Selected mission " << mission_selected << endl;

    if(mission_selected == 1){
        mission = Mission::mission1;
    } else if(mission_selected == 2){
        mission = Mission::mission2;
        cout << "Choose time bonus amount.\n";

        cin >> bonus_selected;

        bonus = bonus_selected;

        cout << "Bonus set to " << bonus << " seconds" << endl;
    }

    if (!savedPath.empty()) {
        cout << "Recovering saved path" << endl;
        path = savedPath;
    } else {

        // Correct borders to account for robot size
        Polygon safeBorders = ClipperHelper::offsetBorders(borders,-1.0 * ROBOT_RADIUS);
        Polygon slottedBorders;

        double xf, yf, thf;    // Endpoint
        double xProj, yProj;   // Projection of the gate center into the borders

        // Compute the gate center and arrival angle, along with the projection
        // of this arrival point into the gate borders
        computeArrival(gate,borders,safeBorders,xf,yf,thf,xProj, yProj);

        if (DO_CUT_GATE_SLOT) {
            slottedBorders = cutGateSlot(gate,safeBorders,xProj, yProj);
        } else {
            // If the gate slot is not cut, the robot can only arrive on the
            // edge of its safe arena to avoid detecting a virtual collision
            // when going for the center of the actual gate
            xf = xProj;
            yf = yProj;
            slottedBorders = safeBorders;
        }

        #ifdef DEBUG_DRAWCURVE
            drawDebugImage(slottedBorders, obstacle_list, victim_list);
        #endif

        vector<dubins::Curve> multipointPath;
        if (mission == Mission::mission1) {
            angle_increment = 2;

            //
            // Sort Victims by ID
            //
            vector<pair<int,Polygon>> orderedVictimList = victim_list;
            sort(orderedVictimList.begin(), orderedVictimList.end(), [](const pair<int,Polygon>& lhs, const pair<int,Polygon>& rhs) {
                return lhs.first < rhs.first;
            });

            //
            // Plan MISSION 1 path
            //
            multipointPath = collectVictimsPath(safeBorders, slottedBorders, obstacle_list, orderedVictimList, x, y, theta, xf, yf, thf, config_folder);

            if (getPathLength(multipointPath) > 0){
            #ifdef DEBUG_SCORES
                cout << "Mission 1 planning completed successfully.\n";
            #endif
            } else {
                throw runtime_error("Mission 1 planning failed!");
            }
        }
        else if (mission == Mission::mission2) {
            angle_increment = 10;
            multipointPath = bestScoreGreedy(safeBorders, slottedBorders, obstacle_list, victim_list, x, y, theta, xf, yf, thf, config_folder);
            
            if (getPathLength(multipointPath) > 0){
            #ifdef DEBUG_SCORES
                cout << "Mission 2 planning completed successfully.\n";
            #endif
            } else {
                throw runtime_error("Mission 2 planning failed!");
            }
        }

        //
        // Path discretization
        //
        vector<Pose> final_path_points;
        double remainingDelta = 0.0;    // value used to carry the remaining "sampling step" between curves
        double last_s = 0.0;            // value used to carry the last value of the curvilinear abscissa
        for (size_t i = 0; i < multipointPath.size(); i++) {
            // the last point of the curve is added only if it's the final curve of the path (otherwise it carries the remaining delta)
            bool addLastPoint = (i == (multipointPath.size()-1) ? true : false);
            // the current curve is sampled with resolution PATH_RESOLUTION
            vector<dubins::Position> res = multipointPath[i].discretizeCurve(PATH_RESOLUTION,remainingDelta,last_s,addLastPoint);

            // Path conversion into compatible output representation
            for (dubins::Position p : res) {
                Pose pose(p.s,p.x,p.y,p.th,p.k);
                final_path_points.push_back(pose);
            }
        }

        //Set output
        path.setPoints(final_path_points);
        savedPath = path;
    }
    return true;
}

}
