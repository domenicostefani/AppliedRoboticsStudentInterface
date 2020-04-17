// color_space_hsv.cpp
// Adapted from OpenCV sample: samples/cpp/tutorial_code/ImgProc/Threshold_inRange.cpp

#include "hsv_panel.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <string>

using namespace std;
using namespace cv;

class WindowData {
public:
    string windowName;
    int low_h, low_s, low_v;
    int high_h, high_s, high_v;
    WindowData(string windowName, int low_h, int low_s, int low_v, int high_h,
               int high_s, int high_v) {
        this->windowName = windowName;
        this->low_h = low_h;
        this->low_s = low_s;
        this->low_v = low_v;
        this->high_h = high_h;
        this->high_s = high_s;
        this->high_v = high_v;
    }
};
/** Function Headers */
void on_low_h_thresh_change(int, void *userdata);
void on_high_h_thresh_change(int, void *userdata);
void on_low_s_thresh_change(int, void *userdata);
void on_high_s_thresh_change(int, void *userdata);
void on_low_v_thresh_change(int, void *userdata);
void on_high_v_thresh_change(int, void *userdata);
void on_ok_button_pressed(int, void *userdata);
WindowData displayControl(const cv::Mat& frame, string windowName,
                          int low_h, int low_s, int low_v,
                          int high_h, int high_s, int high_v);

/*
 * Shows the panels for all the colors, one after the other
 *
 * @param[in] image         bgr image
 * @param[in] out_filename  name of the config file to write
*/
void hsvpanel::show_panel(cv::Mat image, std::string out_filename) {
    /*------------------------------------------------------------------------*/
    /*- Show instructions                                                     */
    /*------------------------------------------------------------------------*/
    /* The next windows are used to set color thresholds                      */
    /* - To skip color tuning press -- Q (quit)                               */
    /* - To advance press ------------ N (next)                               */
    /*          (use the N key also to go through the                         */
    /*          next panels once completed the tuning)                        */
    /*------------------------------------------------------------------------*/
    cv::Mat textframe(300, 700, CV_8UC3, Scalar(245,245,245));
    putText(textframe, "Instructions:", Point2f(10,50),FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0), 2 , 8, false);
    putText(textframe, "The next windows are used to set color thresholds",
          Point2f(10,100),FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1.2 , 8, false);
    putText(textframe, "- To skip color tuning press Q (quit)", Point2f(10,150),
          FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1.2 , 8, false);
    putText(textframe, "- To advance press N (next)", Point2f(10,180),
          FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1.2 , 8, false);
    putText(textframe, "(use the N key also to go through the next panels once completed the tuning)", Point2f(10,230),
          FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1.2 , 8, false);

    namedWindow("Instructions", WINDOW_AUTOSIZE);
        while ((char)waitKey(1)!='n') {
        imshow("Instructions",textframe);
        if ((char)waitKey(1)=='q') {
            destroyWindow("Instructions");
            return;
        }
    }
    destroyWindow("Instructions");

    // Convert image to HSV color space
    cv::Mat img_hsv;
    cv::cvtColor(image, img_hsv, cv::COLOR_BGR2HSV);
    // Open output file
    FILE * fp;
    fp = fopen (out_filename.c_str(),"w");

    /*--------------------------------------------------------------------------*/
    /*      Open the four panels sequentially with custom default values        */
    /*--------------------------------------------------------------------------*/

    string windowName = "Victim Detection (Green)";
    WindowData green = displayControl(img_hsv,windowName,
                                      45,50,50,     //Low HSV default
                                      75,255,255);  //high HSV default

    windowName = "Robot Detection (Blue)";
    WindowData blue = displayControl(img_hsv,windowName,
                                     110,75,10,     //Low HSV default
                                     130,255,255);  //high HSV default

    windowName = "Obstacle Detection1 (LOW red)";
    WindowData lowred = displayControl(img_hsv,windowName,
                                       0,100,100,   //Low HSV default
                                       10,255,255); //high HSV default

    windowName = "Obstacle Detection1 (HIGH red)";
    WindowData highred = displayControl(img_hsv,windowName,
                                        160,100,100, //Low HSV default
                                        180,255,255);//high HSV default

    /*--------------------------------------------------------------------------*/
    /*                             Write output file                            */
    /*--------------------------------------------------------------------------*/

    fprintf(fp, "victims_lowbound %d %d %d\n", green.low_h, green.low_s,
                                               green.low_v);
    fprintf(fp, "victims_highbound %d %d %d\n", green.high_h, green.high_s,
                                                green.high_v);
    fprintf(fp, "robot_lowbound %d %d %d\n", blue.low_h, blue.low_s,
                                             blue.low_v);
    fprintf(fp, "robot_highbound %d %d %d\n", blue.high_h, blue.high_s,
                                              blue.high_v);
    fprintf(fp, "obstacle_lowbound1 %d %d %d\n", lowred.low_h, lowred.low_s,
                                                 lowred.low_v);
    fprintf(fp, "obstacle_highbound1 %d %d %d\n", lowred.high_h, lowred.high_s,
                                                  lowred.high_v);
    fprintf(fp, "obstacle_lowbound2 %d %d %d\n", highred.low_h, highred.low_s,
                                                highred.low_v);
    fprintf(fp, "obstacle_highbound2 %d %d %d\n", highred.high_h, highred.high_s,
                                                  highred.high_v);
    // Close file
    fclose (fp);
}

/*
 * Display single panel with specified parameters
 *
 * @param[in] frame       Input image frame
 * @param[in] windowName  Name to display on window
 * @param[in] low_h       default low Hue
 * @param[in] low_s       default low Saturation
 * @param[in] low_v       default low Value
 * @param[in] high_h      default high Hue
 * @param[in] high_s      default high Saturation
 * @param[in] high_v      default high Value
*/
WindowData displayControl(const cv::Mat& frame, string windowName,
                          int low_h, int low_s, int low_v,
                          int high_h, int high_s, int high_v) {
    bool keepopen = true;                     // flag to keep window open
    cv::Mat frame_threshold;                  // Preview mask with selected values
    namedWindow(windowName, WINDOW_NORMAL);   // Create window
    // Instantiate WindowData element with default parameters
    WindowData wd(windowName,low_h,low_s,low_v,high_h,high_s,high_v);
    /*--------------------------------------------------------------------------*/
    /*                     Create visual controls                               */
    /*--------------------------------------------------------------------------*/
    createTrackbar("Low H" , windowName, &wd.low_h , 180,  on_low_h_thresh_change, (void *)&wd);
    createTrackbar("High H", windowName, &wd.high_h, 180, on_high_h_thresh_change, (void *)&wd);
    createTrackbar("Low S" , windowName, &wd.low_s , 255,  on_low_s_thresh_change, (void *)&wd);
    createTrackbar("High S", windowName, &wd.high_s, 255, on_high_s_thresh_change, (void *)&wd);
    createTrackbar("Low V" , windowName, &wd.low_v , 255,  on_low_v_thresh_change, (void *)&wd);
    createTrackbar("High V", windowName, &wd.high_v, 255, on_high_v_thresh_change, (void *)&wd);

    createButton("OK", on_ok_button_pressed, NULL, CV_PUSH_BUTTON, (void*)&keepopen);
    cvSetWindowProperty("", CV_WND_PROP_AUTOSIZE, CV_WINDOW_NORMAL);

    while (((char)waitKey(1)!='n') && (keepopen == true)) {
        //-- Detect the object based on HSV Range Values
        inRange(frame, Scalar(wd.low_h,wd.low_s,wd.low_v), Scalar(wd.high_h,wd.high_s,wd.high_v), frame_threshold);
        //-- Show the frames
        imshow(windowName,frame_threshold);
    }
    destroyWindow(windowName);
    return wd;
}

void on_low_h_thresh_change(int, void *userdata) {
    WindowData* wd = (WindowData*)userdata;
    wd->low_h = min(wd->high_h-1, wd->low_h);
    setTrackbarPos("Low H", wd->windowName, wd->low_h);
}
void on_high_h_thresh_change(int, void *userdata) {
    WindowData* wd = (WindowData*)userdata;
    wd->high_h = max(wd->high_h, wd->low_h+1);
    setTrackbarPos("High H", wd->windowName, wd->high_h);
}
void on_low_s_thresh_change(int, void *userdata) {
    WindowData* wd = (WindowData*)userdata;
    wd->low_s = min(wd->high_s-1, wd->low_s);
    setTrackbarPos("Low S",wd->windowName, wd->low_s);
}
void on_high_s_thresh_change(int, void *userdata) {
    WindowData* wd = (WindowData*)userdata;
    wd->high_s = max(wd->high_s, wd->low_s+1);
    setTrackbarPos("High S", wd->windowName, wd->high_s);
}
void on_low_v_thresh_change(int, void *userdata) {
    WindowData* wd = (WindowData*)userdata;
    wd->low_v= min(wd->high_v-1, wd->low_v);
    setTrackbarPos("Low V",wd->windowName, wd->low_v);
}
void on_high_v_thresh_change(int, void *userdata) {
    WindowData* wd = (WindowData*)userdata;
    wd->high_v = max(wd->high_v, wd->low_v+1);
    setTrackbarPos("High V", wd->windowName, wd->high_v);
}
void on_ok_button_pressed(int, void*userdata) {
    bool* p_keepopen = (bool*) userdata;
    printf("@on_ok_button_pressed");
    *p_keepopen = false;
}
