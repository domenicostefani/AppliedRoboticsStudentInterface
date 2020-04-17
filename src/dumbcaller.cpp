/* dumbcaller.cpp
 * This is a test script that shows how to call the hsv_panel utility
 *
 * Author: Domenico Stefani
 * Date:   09 January 2020
 *
*/
#include "hsv_panel.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char* argv[]){
    if (argc != 3){
        printf("Usage: %s <image> <outputcsv>\n",argv[0]);
        return 0;
    }
    /*
    The first arg is the name of the input image file that has to be a rgb jpg
    The second output is a target namefile for the csv
    */
    string in_filename = argv[1];
    string out_filename = argv[2];

    /*- Read image -------------------------------------------------------------*/
    cv::Mat in_image = imread(in_filename.c_str());
    if(in_image.empty())
        throw runtime_error("Failed to open file " + in_filename);
    /*- Convert the image in HSV -----------------------------------------------*/
    cvtColor(in_image, in_image, cv::COLOR_BGR2HSV);
    /*- Call the panel routine - -----------------------------------------------*/
    printf("Calling library\n");
    hsvpanel::show_panel(in_image, out_filename);
}