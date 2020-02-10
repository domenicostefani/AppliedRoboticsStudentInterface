#pragma once


#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <string>

namespace hsvpanel{
  void show_panel(cv::Mat image, std::string out_filename);
}
