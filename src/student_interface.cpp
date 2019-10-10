#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <stdlib.h>

#define AUTO_CORNER_DETECTION false

namespace student {
 int i = 0;

 void loadImage(cv::Mat& img_out, const std::string& config_folder){
   throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );
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
    // throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED" );
    std::vector<cv::Point2f> corners;
    if(AUTO_CORNER_DETECTION)
      autodetect_corners(img_in,corners);
    else
      manualselect_corners(img_in,corners);

    // cv::line(img_in, corners[0], corners[1], cv::Scalar(0,0,255));
    // cv::line(img_in, corners[1], corners[2], cv::Scalar(0,0,255));
    // cv::line(img_in, corners[2], corners[3], cv::Scalar(0,0,255));
    // cv::line(img_in, corners[3], corners[0], cv::Scalar(0,0,255));
    //
    // cv::circle(img_in, corners[0], 20, cv::Scalar(50,50,50),4);
    // cv::circle(img_in, corners[1], 20, cv::Scalar(50,50,50),4);
    // cv::circle(img_in, corners[2], 20, cv::Scalar(50,50,50),4);
    // cv::circle(img_in, corners[3], 20, cv::Scalar(50,50,50),4);
    // // display
    // cv::imshow("input", img_in);

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

  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const double scale, const std::string& config_folder){
    // throw std::logic_error( "STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED" );
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED" );
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );
  }


}
