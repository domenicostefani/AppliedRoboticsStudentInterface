#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "dubins.hpp"

#include <experimental/filesystem>
#include <sstream>
#include <stdlib.h>

#define AUTO_CORNER_DETECTION false

namespace student {
 int i = 0;

 void loadImage(cv::Mat& img_out, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );

    static bool initialized = false;
    static std::vector<cv::String> img_list; // list of images to load
    static size_t idx = 0;  // idx of the current img
    static size_t function_call_counter = 0;  // idx of the current img
    const static size_t freeze_img_n_step = 30; // hold the current image for n iteration
    static cv::Mat current_img; // store the image for a period, avoid to load it from file every time

    if(!initialized){
        const bool recursive = false;
        // Load the list of jpg image contained in the config_folder/img_to_load/
        cv::glob(config_folder + "/img_to_load/*.jpg", img_list, recursive);

        if(img_list.size() > 0){
          initialized = true;
          idx = 0;
          current_img = cv::imread(img_list[idx]);
          function_call_counter = 0;
        }else{
          initialized = false;
        }
    }

    if(!initialized){
        throw std::logic_error( "Load Image can not find any jpg image in: " +  config_folder + "/img_to_load/");
        return;
    }

    img_out = current_img;
    function_call_counter++;

    // If the function is called more than N times load increment image idx
    if(function_call_counter > freeze_img_n_step){
        function_call_counter = 0;
        idx = (idx + 1)%img_list.size();
        current_img = cv::imread(img_list[idx]);
    }
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );


    cv::imshow("current picture", img_in);
    char c = cv::waitKey(30);
    if(c == 's'){
      /* Create folder for images*/
      std::string foldername = config_folder;
      foldername += "/image";      //This is the folder used in the topic string
      std::string command = "mkdir -p " + foldername;  //-p creates only if non exists
      system(command.c_str());  //use bash command

      /* Save current image */
      std::string filename = config_folder;
      filename += topic;
      filename += "_";
      filename += std::to_string(i++);
      filename += ".jpg";
      cv::imwrite(filename, img_in);
      printf("Saved is '%s'\n", filename.c_str());
    }
  }

  bool autodetect_corners(const cv::Mat& img_in, std::vector<cv::Point2f>& corners){
      // // convert to grayscale (you could load as grayscale instead)
      // cv::Mat gray;
      // cv::cvtColor(img_in,gray, CV_BGR2GRAY);
      //
      // // compute mask (you could use a simple threshold if the image is always as good as the one you provided)
      // cv::Mat mask;
      // cv::threshold(gray, mask, 10, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
      //
      //
      // // find contours (if always so easy to segment as your image, you could just add the black/rect pixels to a vector)
      // std::vector<std::vector<cv::Point>> contours;
      // std::vector<cv::Vec4i> hierarchy;
      // cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
      //
      // /// Draw contours and find biggest contour (if there are other contours in the image, we assume the biggest one is the desired rect)
      // // drawing here is only for demonstration!
      // int biggestContourIdx = -1;
      // float biggestContourArea = 0;
      // // cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 ); REMOVE
      //
      // for( int i = 0; i< contours.size(); i++ )
      // {
      //     // cv::Scalar color = cv::Scalar(0, 100, 0); REMOVE
      //     //drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() ); REMOVE
      //
      //     float ctArea= cv::contourArea(contours[i]);
      //     if(ctArea > biggestContourArea)
      //     {
      //         biggestContourArea = ctArea;
      //         biggestContourIdx = i;
      //     }
      // }
      //
      // // if no contour found
      // if(biggestContourIdx < 0)
      // {
      //     std::cout << "no contour found" << std::endl;
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
      if(evt == CV_EVENT_LBUTTONDOWN) {
          std::vector<cv::Point>* ptPtr = (std::vector<cv::Point>*)param;
          ptPtr->push_back(cv::Point(x,y));
      }
  }

  void manualselect_corners(const cv::Mat& img_in, std::vector<cv::Point2f>& corners){

    std::vector<cv::Point> points;
    std::string windowname = "Select corners, counterclockwise, start from red";
    cv::namedWindow(windowname);
    cv::setMouseCallback(windowname, onMouse, (void*)&points);

    while(points.size() < 4){
      cv::imshow(windowname, img_in);

      for(int i=0; i < points.size(); i++){
          cv::circle(img_in, points[i], 20, cv::Scalar(240,0,0),CV_FILLED);
      }
      cv::waitKey(1);
    }
    cv::destroyWindow(windowname);

    for(int i=0; i < 4; i++){
        corners.push_back(points[i]);
    }
  }

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points,
                      const cv::Mat& camera_matrix, cv::Mat& rvec,
                      cv::Mat& tvec, const std::string& config_folder){
    // #define ESTRINISIC_CALIB_DEBUG
    std::vector<cv::Point2f> corners;

    if(AUTO_CORNER_DETECTION)
      autodetect_corners(img_in,corners);
    else{
      ///Try to read calibration file

      std::string file_path = config_folder + "/extrinsicCalib.csv";

      if (!std::experimental::filesystem::exists(file_path)){
        // File does not exist
        manualselect_corners(img_in,corners);
        // Save the file
        std::experimental::filesystem::create_directories(config_folder);
        std::ofstream output(file_path);
        if (!output.is_open()){
          throw std::runtime_error("Cannot write file: " + file_path);
        }
        for (const auto pt: corners) {
          output << pt.x << " " << pt.y << std::endl;
        }
        output.close();
      }else{
        // Load configuration from file
        std::ifstream input(file_path);
        if (!input.is_open()){
          throw std::runtime_error("Cannot read file: " + file_path);
        }
        while (!input.eof()){
          double x, y;
          if (!(input >> x >> y)) {
            if (input.eof()) break;
            else {
              throw std::runtime_error("Malformed file: " + file_path);
            }
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
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out,
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    cv::undistort(img_in,img_out,cam_matrix,dist_coeffs);
    // throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );

  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec,
                          const cv::Mat& tvec,
                          const std::vector<cv::Point3f>& object_points_plane,
                          const std::vector<cv::Point2f>& dest_image_points_plane,
                          cv::Mat& plane_transf, const std::string& config_folder){

    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
  }

  void unwarp(const cv::Mat& img_in, cv::Mat& img_out,
              const cv::Mat& transf, const std::string& config_folder){
    // throw std::logic_error( "STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED" );
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }

  void findObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){
     #define FIND_OBSTACLES_DEBUG

     /* Red color requires 2 ranges*/
     cv::Mat lower_red_hue_range; // the lower range for red hue
     cv::Mat upper_red_hue_range; // the higher range for red hue
     cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
     cv::inRange(hsv_img, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);
     // Now we can combine the 2 masks
     cv::Mat red_hue_image;
     cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
     //This can reduce false positives
     // cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

     #ifdef FIND_OBSTACLES_DEBUG
       //imshow("Red filtered",red_hue_image);
       //cv::waitKey(0);
     #endif

     /*
      * Now the mask has to undergo contour detection
     */

     std::vector<std::vector<cv::Point>> contours, contours_approx;
     std::vector<cv::Point> approx_curve;
     cv::Mat contours_img;
     contours_img = hsv_img.clone();

     cv::findContours(red_hue_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

     drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA); //REMOVE
     for (int i=0; i<contours.size(); ++i)
     {
       approxPolyDP(contours[i], approx_curve, 3, true);

       Polygon scaled_contour;
       for (const auto& pt: approx_curve) {
         scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
       }
       obstacle_list.push_back(scaled_contour);

       contours_approx = {approx_curve};
       cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }

    imshow("obstacles",contours_img);
    cv::waitKey(0);
  }

  bool findGate(const cv::Mat& hsv_img, const double scale, Polygon& gate){

    // Find green regions
    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 50), cv::Scalar(75, 255, 255), green_mask);

    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;
    contours_img = hsv_img.clone();

    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool res = false;

    for( auto& contour : contours){
        //const double area = cv::contourArea(contour);
        //std::cout << "AREA " << area << std::endl;
        //std::cout << "SIZE: " << contours.size() << std::endl;

        approxPolyDP(contour, approx_curve, 10, true);

        if (approx_curve.size() != 4) continue;

        contours_approx = {approx_curve};
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

        for (const auto& pt: approx_curve) {
          gate.emplace_back(pt.x/scale, pt.y/scale);
        }

        res = true;
        break;
    }

    cv::imshow("findGate", contours_img);
    cv::waitKey(0);

    return res;
  }

  bool findVictims(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list){

    // Find green regions
    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 50), cv::Scalar(75, 255, 255), green_mask);

    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;
    contours_img = hsv_img.clone();

    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i=0; i<contours.size(); ++i)
    {
        approxPolyDP(contours[i], approx_curve, 10, true);

        if( approx_curve.size() != 4){  // ignore gate

          Polygon scaled_contour;
          for (const auto& pt: approx_curve) {
            scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
          }
          victim_list.push_back({i+1, scaled_contour});

          contours_approx = {approx_curve};
          drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
        }
    }

    cv::imshow("findVictims", contours_img);
    cv::waitKey(0);

    return true;
  }

  bool processMap(const cv::Mat& img_in, const double scale,
                  std::vector<Polygon>& obstacle_list,
                  std::vector<std::pair<int,Polygon>>& victim_list,
                  Polygon& gate, const std::string& config_folder){

      // Convert to HSV for better color detection
      cv::Mat img_hsv;
      cv::cvtColor(img_in, img_hsv, cv::COLOR_BGR2HSV);

      findGate(img_hsv, scale, gate);
      findVictims(img_hsv, scale, victim_list);
      findObstacles(img_hsv, scale, obstacle_list);

      return true;
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );
    #define FIND_ROBOT_DEBUG_PLOT

    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    // Extract blue color region
    cv::Mat blue_mask;
    cv::inRange(hsv_img, cv::Scalar(110, 75, 0), cv::Scalar(130, 255, 255), blue_mask);

    // remove noise with opening operation
    //cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

    // find robot contours and approximate to triangle
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    #ifdef FIND_ROBOT_DEBUG_PLOT // do this only if FIND_DEBUG_PLOT is defined
      //cv::imshow("findRobotHsv", hsv_img);
      //cv::imshow("findRobotMask", blue_mask);
      cv::Mat contours_img;
      contours_img = img_in.clone();
      //cv::drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
      std::cout << "Number of contours: " << contours.size() << std::endl;
    #endif

    std::vector<cv::Point> approx_curve;
    std::vector<std::vector<cv::Point>> contours_approx;
    bool found = false;
    for (int i=0; i<contours.size(); ++i)
    {
        // Approximate the i-th contours
        cv::approxPolyDP(contours[i], approx_curve, 10, true);

        // Check the number of edge of the aproximated contour
        if (approx_curve.size() != 3) continue;

        // draw approximated contours
        #ifdef FIND_ROBOT_DEBUG_PLOT
            //std::cout << "Approx contour count: " << approx_curve.size() << std::endl;
            contours_approx = {approx_curve};
            cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,0,255), 1, cv::LINE_AA);
        #endif

        cv::imshow("findRobot", contours_img);
        cv::waitKey(0);

        found = true;
        break;
    }

    // set robot position
    if (found){
        // emplace back every vertex on triangle (output of this function)
        for (const auto& pt: approx_curve) {
          triangle.emplace_back(pt.x/scale, pt.y/scale);
          // remember to use the scale to convert the position on the image
          // (pixels) to the position in the arena (meters)
        }

        // Find the position of the robot (baricenter of the triangle)
        double cx = 0, cy = 0;

        // Compute the triangle baricenter
        for (auto vertex: triangle)
        {
          // NB: triangle point are expressed in meters
          cx += vertex.x;
          cy += vertex.y;
        }
        cx /= static_cast<double>(triangle.size());
        cy /=  static_cast<double>(triangle.size());

        // Find the robot orientation (i.e the angle of height relative to the base with the x axis)
        double dst = 0;
        Point top_vertex;
        for (auto& vertex: triangle)
        {
          const double dx = vertex.x-cx;
          const double dy = vertex.y-cy;
          const double curr_d = dx*dx + dy*dy;
          if (curr_d > dst)
          {
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
        theta = std::atan2(dy, dx);

        #ifdef FIND_ROBOT_DEBUG_PLOT
            // Draw over the image
            cv::Point cv_baricenter(x*scale, y*scale); // convert back m to px
            cv::Point cv_vertex(top_vertex.x*scale, top_vertex.y*scale); // convert back m to px
            cv::line(contours_img, cv_baricenter, cv_vertex, cv::Scalar(0,255,0), 3);
            cv::circle(contours_img, cv_baricenter, 5, cv::Scalar(0,0,255), -1);
            cv::circle(contours_img, cv_vertex, 5, cv::Scalar(0,255,0), -1);
            std::cout << "(x, y, theta) = " << x << ", " << y << ", " << theta*180/M_PI << std::endl;
        #endif
    }

    #ifdef FIND_ROBOT_DEBUG_PLOT
      cv::imshow("findRobot", contours_img);
      cv::waitKey(0);
    #endif

  return found;
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,
                const std::vector<std::pair<int,Polygon>>& victim_list,
                const Polygon& gate, const float x, const float y, const float theta,
                Path& path,
                const std::string& config_folder){

    printf("--------PLANNING WAS CALLED--------\n");

    // throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );

    //Test code
    double x0 = 0.0;
    double y0 = 0.0;
    double th0 = -2.0 / 3 * M_PI;
    double xf = 4.0;
    double yf = 0.0;
    double thf = M_PI / 3.0;
    double Kmax = 3.0;

    int pidx;

    dubins::Curve curve = dubins::dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax, pidx);

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

    return true;
  }
}
