/*
 * Arena Corner Detection
 *
*/
#pragma once

namespace CornerDetection{

cv::Point findLineCenter(const cv::Mat& img_in, const std::vector<cv::Point> &arena);
void onMouse(int evt, int x, int y, int flags, void* param);
void readSelection(const cv::Mat& img_in, std::vector<cv::Point2f>& corners);

/**
 * Detect automatically the arena corners
*/
std::vector<cv::Point2f> autodetect(const cv::Mat& img_in) {
    std::vector<cv::Point2f> corners;

    // convert to grayscale
    cv::Mat gray;
    cv::cvtColor(img_in,gray, CV_BGR2GRAY);
    // compute mask
    cv::Mat mask;
    cv::threshold(gray, mask, 120, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    // find contours (if always so easy to segment as your image, you could just add the black/rect pixels to a std::vector)
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // Find biggest contour (if there are other contours in the image, we assume
    // the biggest one is the desired rect)
    int biggestContourIdx = -1;
    float biggestContourArea = 0;

    #ifdef DEBUG_CORNER_AUTODETECT
        cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 ); //Todo: REMOVE
    #endif

    for (size_t i = 0; i < contours.size(); i++) {
        float ctArea= cv::contourArea(contours[i]);
        if(ctArea > biggestContourArea)
        {
            biggestContourArea = ctArea;
            biggestContourIdx = i;
        }
    }

    #ifdef DEBUG_CORNER_AUTODETECT
        cv::Mat display = img_in.clone();
        cv::Scalar color = cv::Scalar(0, 0, 255); // Todo: REMOVE
        cv::drawContours( display, contours,biggestContourIdx , color, 1, 8, hierarchy, 0, cv::Point() );
        cv::imshow("C3",display);
        cv::waitKey(0);
    #endif

    std::vector<cv::Point> approx_curve;
    approxPolyDP(contours[biggestContourIdx], approx_curve,4, true);
    // Now that the four corners are found they have to be ordered
    // The first is the one closer to the red line
    // Then counterclockwise order

    //
    // Find Red line
    //                /* Red color requires 2 ranges*/
    cv::Point redLine = findLineCenter(img_in,approx_curve);
    //
    // Find nearestCorner
    //
    float lowestDistance = 1000000;
    int nearestCorner = -1;
    for (int i = 0; i < 4; i++) {
        float distance = pow((approx_curve[i].x-redLine.x),2) + pow((approx_curve[i].y-redLine.y),2);
        if (distance < lowestDistance) {
            lowestDistance = distance;
            nearestCorner = i;
        }
    }
    assert((nearestCorner >= 0)&&(nearestCorner <= 3));

    //
    // Order Corners
    //
    for (int i = 0; i < 4; ++i) {
        int index = (nearestCorner + i) % 4;
        corners.emplace_back(approx_curve[index].x,approx_curve[index].y);
    }

    return corners;
}

/**
 * Read the configuration, if not existent ask the user to select points
*/
std::vector<cv::Point2f> manualSelect(const cv::Mat& img_in, std::string config_folder) {
    //
    //  Read configuration file or ask for manual selection
    //
    std::vector<cv::Point2f> corners;
    ///Try to read calibration file
    std::string file_path = config_folder + "/extrinsicCalib.csv";

    if (!std::experimental::filesystem::exists(file_path)) {
        // File does not exist
        CornerDetection::readSelection(img_in,corners);
        // Save the file
        std::experimental::filesystem::create_directories(config_folder);
        std::ofstream output(file_path);
        if (!output.is_open()) {
            throw std::runtime_error("Cannot write file: " + file_path);
        }
        for (const auto pt: corners) {
            output << pt.x << " " << pt.y << std::endl;
        }
        output.close();
    } else {
        // Load configuration from file
        std::ifstream input(file_path);
        if (!input.is_open()) {
            throw std::runtime_error("Cannot read file: " + file_path);
        }
        while (!input.eof()) {
            double x, y;
            if (!(input >> x >> y)) {
                if (input.eof())
                    break;
                else
                  throw std::runtime_error("Malformed file: " + file_path);
            }
            corners.emplace_back(x, y);
        }
        input.close();
    }
    return corners;
}

/**
 * Find the center of the redline in the arena
*/
cv::Point findLineCenter(const cv::Mat& img_in, const std::vector<cv::Point> &arena) {
    cv::Mat lower_red_hue_range; // the lower range for red hue
    cv::Mat upper_red_hue_range; // the higher range for red hue
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
    cv::inRange(hsv_img, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), upper_red_hue_range);
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

    cv::Size size(red_hue_image.cols, red_hue_image.rows);
    cv::Mat maskRect(size, CV_8U, cv::Scalar(0.0));

    std::vector<cv::Point2f> tempcorners;
    for (int i = 0; i < 4; ++i)
        tempcorners.emplace_back(arena[i].x, arena[i].y);
    cv::Scalar white = cv::Scalar(255.0); // white
    cv::Point vertices[4];
    for (int i = 0; i < 4; ++i) {
        vertices[i] = tempcorners[i];
    }
    cv::fillConvexPoly(maskRect, vertices, 4, white);

    // compute mask
    cv::Mat maskContours;
    cv::threshold(red_hue_image, maskContours, 120, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    cv::subtract(red_hue_image, maskRect, maskContours);

    std::vector<std::vector<cv::Point>> contoursForLine;
    cv::findContours(maskContours,contoursForLine, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // Find biggest contour (if there are other contours in the image, we assume
    // the biggest one is the desired rect)
    int biggestContourIdxForLine = -1;
    float biggestContourAreaForLine = 0;

    for (size_t i = 0; i< contoursForLine.size(); i++) {
        float ctArea= cv::contourArea(contoursForLine[i]);
        if(ctArea > biggestContourAreaForLine)
        {
            biggestContourAreaForLine = ctArea;
            biggestContourIdxForLine = i;
        }
    }
    std::vector<cv::Point> approx_curve_forLine;
    approxPolyDP(contoursForLine[biggestContourIdxForLine], approx_curve_forLine,4, true);

    int xsum = 0;
    int ysum = 0;
    for (int i = 0; i < 4; i++) {
        xsum += approx_curve_forLine[i].x;
        ysum += approx_curve_forLine[i].y;
    }
    cv::Point lineCenter(xsum/4.0,ysum/4.0);

    return lineCenter;
}

/**
 * Event handler for mouse event
*/
void onMouse(int evt, int x, int y, int flags, void* param) {
    if (evt == CV_EVENT_LBUTTONDOWN) {
        std::vector<cv::Point>* ptPtr = (std::vector<cv::Point>*)param;
        ptPtr->push_back(cv::Point(x,y));
    }
}

/**
 * Ask the user to manually select the corners in order
*/
void readSelection(const cv::Mat& img_in, std::vector<cv::Point2f>& corners) {
    std::vector<cv::Point> points;
    std::string windowname = "Select corners, counterclockwise, start from red";
    cv::namedWindow(windowname);
    cv::setMouseCallback(windowname, onMouse, (void*)&points);

    while (points.size() < 4) {
        cv::imshow(windowname, img_in);

        for (size_t i=0; i < points.size(); i++) {
            cv::circle(img_in, points[i], 20, cv::Scalar(240,0,0),CV_FILLED);
        }
        cv::waitKey(1);
    }
    cv::destroyWindow(windowname);

    for (int i=0; i < 4; i++) {
        corners.push_back(points[i]);
    }
}

}