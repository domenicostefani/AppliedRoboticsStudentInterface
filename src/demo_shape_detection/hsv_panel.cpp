// color_space_hsv.cpp
// Adapted from OpenCV sample: samples/cpp/tutorial_code/ImgProc/Threshold_inRange.cpp

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <string>

#define SHOW_ORIGINAL false

using namespace std;
using namespace cv;

class WindowData{
public:
  string windowName;
  int low_h, low_s, low_v;
  int high_h, high_s, high_v;
  WindowData(string windowName, int low_h, int low_s, int low_v, int high_h, int high_s, int high_v){
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
void on_low_h_thresh_trackbar(int, void *userdata);
void on_high_h_thresh_trackbar(int, void *userdata);
void on_low_s_thresh_trackbar(int, void *userdata);
void on_high_s_thresh_trackbar(int, void *userdata);
void on_low_v_thresh_trackbar(int, void *userdata);
void on_high_v_thresh_trackbar(int, void *userdata);
void on_ok_button_pressed(int, void *userdata);
WindowData displayControl(string in_filename, string windowName,int low_h, int low_s,
                    int low_v, int high_h, int high_s, int high_v);

int main(int argc, char* argv[]){
  if (argc != 3){
      printf("Usage: %s <image> <outputcsv>\n",argv[0]);
      return 0;
  }
  string in_filename = argv[1];
  string out_filename = argv[2];

  FILE * fp;
  fp = fopen (out_filename.c_str(),"w");

  string windowName = "Victim Detection (Green)";
  WindowData green = displayControl(in_filename,windowName,45,50,50,75,255,255);

  windowName = "Robot Detection (Blue)";
  WindowData blue = displayControl(in_filename,windowName,110,75,10,130,255,255);

  windowName = "Obstacle Detection1 (LOW red)";
  WindowData lowred = displayControl(in_filename,windowName,0,100,100,10,255,255);

  windowName = "Obstacle Detection1 (HIGH red)";
  WindowData highred = displayControl(in_filename,windowName,160,100,100,180,255,255);

  fprintf(fp, "victims_lowbound,%d,%d,%d\n", green.low_h, green.low_s, green.low_v);
  fprintf(fp, "victims_highbound,%d,%d,%d\n", green.high_h, green.high_s, green.high_v);
  fprintf(fp, "robot_lowbound,%d,%d,%d\n", blue.low_h, blue.low_s, blue.low_v);
  fprintf(fp, "robot_highbound,%d,%d,%d\n", blue.high_h, blue.high_s, blue.high_v);
  fprintf(fp, "obstacle_lowbound1,%d,%d,%d\n", lowred.low_h, lowred.low_s, lowred.low_v);
  fprintf(fp, "obstacle_highbound1,%d,%d,%d\n", lowred.high_h, lowred.high_s, lowred.high_v);
  fprintf(fp, "obstacle_lowbound2,%d,%d,%d\n", highred.low_h, highred.low_s, highred.low_v);
  fprintf(fp, "obstacle_highbound2,%d,%d,%d\n", highred.high_h, highred.high_s, highred.high_v);

  fclose (fp);
  return 0;
}

WindowData displayControl(string in_filename, string windowName,int low_h, int low_s,
                    int low_v, int high_h, int high_s, int high_v){
  bool keepopen = true;
  Mat frame, frame_threshold;
  if(SHOW_ORIGINAL) namedWindow("Original Image", WINDOW_NORMAL);
  namedWindow(windowName, WINDOW_NORMAL);

  WindowData wd(windowName,low_h,low_s,low_v,high_h,high_s,high_v);

  createTrackbar("Low H" , windowName, &wd.low_h, 180, on_low_h_thresh_trackbar,
                 (void *)&wd);
  createTrackbar("High H", windowName, &wd.high_h, 180, on_high_h_thresh_trackbar,
                 (void *)&wd);
  createTrackbar("Low S" , windowName, &wd.low_s, 255, on_low_s_thresh_trackbar,
                 (void *)&wd);
  createTrackbar("High S", windowName, &wd.high_s, 255, on_high_s_thresh_trackbar,
                 (void *)&wd);
  createTrackbar("Low V" , windowName, &wd.low_v, 255, on_low_v_thresh_trackbar,
                 (void *)&wd);
  createTrackbar("High V", windowName, &wd.high_v, 255, on_high_v_thresh_trackbar,
                 (void *)&wd);

  createButton("OK", on_ok_button_pressed, NULL, CV_PUSH_BUTTON, (void*)&keepopen);
  cvSetWindowProperty("", CV_WND_PROP_AUTOSIZE, CV_WINDOW_NORMAL);

  frame = imread(in_filename.c_str());
  if(frame.empty())
      throw runtime_error("Failed to open file " + in_filename);

  cvtColor(frame, frame, cv::COLOR_BGR2HSV);
  if(SHOW_ORIGINAL) imshow("Original Image", frame);

  while(((char)waitKey(1)!='q') && (keepopen == true)){
      //-- Detect the object based on HSV Range Values
      inRange(frame, Scalar(wd.low_h,wd.low_s,wd.low_v), Scalar(wd.high_h,wd.high_s,wd.high_v), frame_threshold);
      //-- Show the frames
      imshow(windowName,frame_threshold);
  }
  destroyWindow(windowName);
  return wd;
}

void on_low_h_thresh_trackbar(int, void *userdata){
  WindowData* wd = (WindowData*)userdata;
  wd->low_h = min(wd->high_h-1, wd->low_h);
  setTrackbarPos("Low H", wd->windowName, wd->low_h);
}
void on_high_h_thresh_trackbar(int, void *userdata){
  WindowData* wd = (WindowData*)userdata;
  wd->high_h = max(wd->high_h, wd->low_h+1);
  setTrackbarPos("High H", wd->windowName, wd->high_h);
}
void on_low_s_thresh_trackbar(int, void *userdata){
  WindowData* wd = (WindowData*)userdata;
  wd->low_s = min(wd->high_s-1, wd->low_s);
  setTrackbarPos("Low S",wd->windowName, wd->low_s);
}
void on_high_s_thresh_trackbar(int, void *userdata){
  WindowData* wd = (WindowData*)userdata;
  wd->high_s = max(wd->high_s, wd->low_s+1);
  setTrackbarPos("High S", wd->windowName, wd->high_s);
}
void on_low_v_thresh_trackbar(int, void *userdata){
  WindowData* wd = (WindowData*)userdata;
  wd->low_v= min(wd->high_v-1, wd->low_v);
  setTrackbarPos("Low V",wd->windowName, wd->low_v);
}
void on_high_v_thresh_trackbar(int, void *userdata){
  WindowData* wd = (WindowData*)userdata;
  wd->high_v = max(wd->high_v, wd->low_v+1);
  setTrackbarPos("High V", wd->windowName, wd->high_v);
}
void on_ok_button_pressed(int, void*userdata){
  bool* p_keepopen = (bool*) userdata;
  printf("@on_ok_button_pressed");
  *p_keepopen = false;
}
