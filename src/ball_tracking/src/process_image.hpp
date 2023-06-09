#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>





/*
This function takes an input image and a window and applies the window to the image, returning the masked image.
The window is defined as a vector of four floats representing percentages of the image dimensions from left, top, right, and bottom.
The function first calculates the pixel coordinates of the window based on the image dimensions and the input window.
Then, it initializes a mask as a black image with the same size and type as the input image.
It copies the pixels from the original image corresponding to the window to the mask.
Finally, the function returns the masked image.
*/
cv::Mat apply_search_window(cv::Mat image, std::vector<int> window_adim={0, 0, 1, 1})
{
    int rows = image.rows;
    int cols = image.cols;
    int x_min_px = cols * window_adim[0] / 100.0;
    int y_min_px = rows * window_adim[1] / 100.0;
    int x_max_px = cols * window_adim[2] / 100.0;
    int y_max_px = rows * window_adim[3] / 100.0;

    // Initialize the mask as a black image
    cv::Mat mask = cv::Mat::zeros(image.size(), image.type());

    // Copy the pixels from the original image corresponding to the window
    image(cv::Rect(x_min_px, y_min_px, x_max_px - x_min_px, y_max_px - y_min_px)).copyTo(mask(cv::Rect(x_min_px, y_min_px, x_max_px - x_min_px, y_max_px - y_min_px)));

    // return the mask
    return mask;
}





/*
This function takes an input image and a rectangle in pixel units, and draws a rectangle on the image with the given color and line thickness.
It uses OpenCV's rectangle function to draw the rectangle on the image.
Finally, it returns the updated image.
*/
cv::Mat draw_window2(cv::Mat image,         // Input image
                     std::vector<int> rect_px,  // Window in pixel units
                     cv::Scalar color=cv::Scalar(255,0,0),  // Line's color
                     int line=5  // Line's thickness
                    )
{
    // Draw a rectangle from top left to bottom right corner using OpenCV's rectangle function
    cv::rectangle(image, cv::Point(rect_px[0], rect_px[1]), cv::Point(rect_px[2], rect_px[3]), color, line);

    // Return the updated image
    return image;
}





/*
This function converts a rectangle defined in percentage coordinates to pixel coordinates based on the dimensions of an input image.
The input rectangle is defined as a vector of four floats representing percentages of the image dimensions from left, top, right, and bottom.
The function first calculates the pixel coordinates of the rectangle by multiplying the percentage coordinates by the image dimensions.
Then, it returns the pixel coordinates of the rectangle.
*/
std::vector<int> convert_rect_perc_to_pixels(std::vector<double> rect_perc, cv::Mat image) {
    
    int rows = image.rows; // Get the number of rows in the image
    int cols = image.cols; // Get the number of columns in the image

    std::vector<int> scale = {cols, rows, cols, rows}; // Create a vector containing the scale for each rectangle coordinate

    std::vector<int> rect_px(4); // Create a vector to hold the pixel coordinates of the rectangle

    // Convert each percentage coordinate to pixels
    for(int i = 0; i < 4; i++) {
        rect_px[i] = double(rect_perc[i]*scale[i]/100);
    }
    
    return rect_px; // Return the pixel coordinates of the rectangle
}





/* 
This function takes an input OpenCV image and a keypoint and returns a new keypoint with normalized coordinates and size.
The keypoint's x and y coordinates are normalized to the center of the image, such that the center of the image corresponds to (0,0) and the edges of the image correspond to (-1,-1) and (1,1) respectively.
The size of the keypoint is normalized by dividing it by the width of the image.
The function returns the normalized keypoint.
*/
cv::KeyPoint normalize_keypoint(cv::Mat cv_image, cv::KeyPoint kp) {
    float rows = static_cast<float>(cv_image.rows);
    float cols = static_cast<float>(cv_image.cols);
    float center_x = 0.5*cols;
    float center_y = 0.5*rows;
    float x = (kp.pt.x - center_x)/(center_x);
    float y = (kp.pt.y - center_y)/(center_y);
    // Size of the keypoint is normalized by dividing it by the width of the image
    float size = kp.size/cv_image.cols;
    return cv::KeyPoint(x, y, size);
}





/*
This function takes in an input image and the tuning parameters and uses OpenCV functions to perform object detection on the image.
The function performs object detection using an OpenCV SimpleBlobDetector.
The function then returns a tuple containing the detected keypoints, the output image, and the tuning image.
*/
std::tuple<std::vector<cv::KeyPoint>, cv::Mat, cv::Mat> find_circles(cv::Mat image, std::map<std::string, int> tuning_params) {

    int blur = 5;

    double x_min = tuning_params["x_min"];
    double x_max = tuning_params["x_max"];
    double y_min = tuning_params["y_min"];
    double y_max = tuning_params["y_max"];
    
    std::vector<double> search_window {x_min, y_min, x_max, y_max};

    cv::Mat blurred_image;
    cv::blur(image, blurred_image, cv::Size(blur, blur));
    cv::Mat working_image = blurred_image.clone();


    // Search window
    if (search_window.empty()) {
        search_window = {0.0, 0.0, 1.0, 1.0};
    }
    std::vector<int> search_window_px = convert_rect_perc_to_pixels(search_window, image);


    // Convert image from BGR to HSV
    cv::cvtColor(working_image, working_image, cv::COLOR_BGR2HSV);

    // Apply HSV threshold to create binary mask (where pixels outside HSV thresholds are colored black and pixels within HSV thresholds are colored white)
    cv::Scalar thresh_min(tuning_params["h_min"], tuning_params["s_min"], tuning_params["v_min"]);
    cv::Scalar thresh_max(tuning_params["h_max"], tuning_params["s_max"], tuning_params["v_max"]);
    cv::inRange(working_image, thresh_min, thresh_max, working_image);


    // Dilate and Erode
    cv::dilate(working_image, working_image, cv::Mat(), cv::Point(-1,-1), 2);
    cv::erode(working_image, working_image, cv::Mat(), cv::Point(-1,-1), 2);


    // Make a copy of the image for tuning
    cv::Mat tuning_image = cv::Mat::zeros(image.size(), CV_8UC3);
    cv::bitwise_and(image, image, tuning_image, working_image);


    // Apply the search window
    working_image = apply_search_window(working_image, search_window_px);


    // Invert the image to suit the blob detector
    working_image = 255 - working_image;



    // Initialize parameters for SimpleBlobDetector
    cv::SimpleBlobDetector::Params params;
        
    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;
        
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 30;
    params.maxArea = 20000;
        
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.1;
        
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.5;
        
    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5;

    // Create a SimpleBlobDetector with the specified parameters
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);


    // Run detection
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(working_image, keypoints);

    // Define size limits in pixels
    float size_min_px = tuning_params["sz_min"] * working_image.cols / 100.0;
    float size_max_px = tuning_params["sz_max"] * working_image.cols / 100.0;

    // Remove keypoints smaller or larger than specified sizes
    for (auto it = keypoints.begin(); it != keypoints.end();) {
        if (it->size < size_min_px || it->size > size_max_px) {
            it = keypoints.erase(it);
        } else {
            it++;
        }
    }

    // Set up main output image
    cv::Scalar line_color(0, 0, 255);
    cv::Mat output_image;
    cv::drawKeypoints(image, keypoints, output_image, line_color, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    output_image = draw_window2(output_image, search_window_px);

    // Set up tuning output image
    cv::drawKeypoints(tuning_image, keypoints, tuning_image, line_color, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    tuning_image = draw_window2(tuning_image, search_window_px);

    // Normalize keypoints based on the size of the image
    std::vector<cv::KeyPoint> keypoints_normalized;
    for (auto k : keypoints) {
        keypoints_normalized.push_back(normalize_keypoint(working_image, k));
    }


    return std::make_tuple(keypoints_normalized, output_image, tuning_image);
}





// Define a function to wait for a short period of time, allowing GUI events to be processed
void wait_on_gui() {
    cv::waitKey(2);
}