/** \file hsv_panel.hpp
 * @brief HSV Panel library.
 *
 * This library allow to show a configuration panel for the Hue-Saturation-Value
 * ranges for different colors of differen elements.
 * Configurations are saved on a text file that can later be read.
*/
#pragma once

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <string>

//! Color configuration panel utilities
namespace hsvpanel{
    /** show the hsv tuning panel.
     * This shows a panel to tune the different Hue-Saturation-value ranges for
     * image elaboration and element detection.
     *
     * @param[in] image         bgr image
     * @param[in] out_filename  name of the config file to write to
    */
    void show_panel(cv::Mat image, std::string out_filename);
}
