#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>





std::tuple<std::vector<cv::KeyPoint>, cv::Mat, cv::Mat> find_circles(cv::Mat image, std::map<std::string, int> tuning_params) {

    int blur = 5;

    int x_min = tuning_params["x_min"];
    int x_max = tuning_params["x_max"];
    int y_min = tuning_params["y_min"];
    int y_max = tuning_params["y_max"];
    
    std::vector<int> search_window {x_min, y_min, x_max, y_max};

    cv::Mat working_image = cv::blur(image, cv::Size(blur, blur));


    // Search window
    if (search_window.empty()) {
        search_window = {0, 0, 1, 1};
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
    cv::Mat tuning_image;
    cv::drawKeypoints(tuning_image, keypoints, tuning_image, line_color, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    tuning_image = draw_window2(tuning_image, search_window_px);

    // Normalize keypoints based on the size of the image
    std::vector<cv::KeyPoint> keypoints_normalized;
    for (auto k : keypoints) {
        keypoints_normalized.push_back(normalize_keypoint(working_image, k));
    }


    return std::make_tuple(keypoints_normalised, output_image, tuning_image);
}





/*
This function takes an input image and a window and applies the window to the image, returning the masked image.
The window is defined as a vector of four floats representing percentages of the image dimensions from left, top, right, and bottom.
The function first calculates the pixel coordinates of the window based on the image dimensions and the input window.
Then, it initializes a mask as a black image with the same size and type as the input image.
It copies the pixels from the original image corresponding to the window to the mask.
Finally, the function returns the masked image.
*/
cv::Mat apply_search_window(cv::Mat image, std::vector<float> window_adim={0.0, 0.0, 1.0, 1.0})
{
    int rows = image.rows;
    int cols = image.cols;
    int x_min_px = cols * window_adim[0] / 100;
    int y_min_px = rows * window_adim[1] / 100;
    int x_max_px = cols * window_adim[2] / 100;
    int y_max_px = rows * window_adim[3] / 100;

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
std::vector<int> convert_rect_perc_to_pixels(std::vector<float> rect_perc, cv::Mat image) {
    
    int rows = image.rows; // Get the number of rows in the image
    int cols = image.cols; // Get the number of columns in the image

    std::vector<int> scale = {cols, rows, cols, rows}; // Create a vector containing the scale for each rectangle coordinate

    std::vector<int> rect_px(4); // Create a vector to hold the pixel coordinates of the rectangle

    // Convert each percentage coordinate to pixels
    for(int i = 0; i < 4; i++) {
        rect_px[i] = static_cast<int>(rect_perc[i]*scale[i]/100);
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





// Define a no-op function to use as a callback for OpenCV trackbars
void no_op(int x) {
    // Do nothing
}





/*
This function creates a tuning window with multiple trackbars to adjust the values of different parameters for image processing.
The function takes an input map containing initial values for each trackbar.
The window is named "Tuning", and each trackbar is named with a string corresponding to the parameter it represents.
The function creates each trackbar with the specified initial value, maximum value, and a no-op function.
*/
void create_tuning_window(std::map<std::string, int> initial_values) {
    // Create a window with the name "Tuning"
    cv::namedWindow("Tuning", 0);

    // Create trackbars with their names, window name, initial value, maximum value, and a no_op function
    cv::createTrackbar("x_min", "Tuning", &(initial_values["x_min"]), 100, no_op);
    cv::createTrackbar("x_max", "Tuning", &(initial_values["x_max"]), 100, no_op);
    cv::createTrackbar("y_min", "Tuning", &(initial_values["y_min"]), 100, no_op);
    cv::createTrackbar("y_max", "Tuning", &(initial_values["y_max"]), 100, no_op);
    cv::createTrackbar("h_min", "Tuning", &(initial_values["h_min"]), 180, no_op);
    cv::createTrackbar("h_max", "Tuning", &(initial_values["h_max"]), 180, no_op);
    cv::createTrackbar("s_min", "Tuning", &(initial_values["s_min"]), 255, no_op);
    cv::createTrackbar("s_max", "Tuning", &(initial_values["s_max"]), 255, no_op);
    cv::createTrackbar("v_min", "Tuning", &(initial_values["v_min"]), 255, no_op);
    cv::createTrackbar("v_max", "Tuning", &(initial_values["v_max"]), 255, no_op);
    cv::createTrackbar("sz_min", "Tuning", &(initial_values["sz_min"]), 100, no_op);
    cv::createTrackbar("sz_max", "Tuning", &(initial_values["sz_max"]), 100, no_op);
}





/*
This function retrieves the current values of trackbars created in a tuning window.
The trackbar names are stored in a vector called "trackbar_names", and the tuning window is named "Tuning".
The function creates an empty map called "tuning_params" to store the trackbar values.
It then iterates through each trackbar name in the "trackbar_names" vector, retrieves the current value of the trackbar using OpenCV's "getTrackbarPos" function, and stores the value in the "tuning_params" map with the corresponding trackbar name as the key.
The function then returns the "tuning_params" map containing the current values of each trackbar.
*/
std::map<std::string, int> get_tuning_params() {
    // Define the names of the trackbars to read from
    std::vector<std::string> trackbar_names = {"x_min", "x_max", "y_min", "y_max", "h_min", "h_max", "s_min", "s_max", "v_min", "v_max", "sz_min", "sz_max"};

    // Define a map to store the trackbar values
    std::map<std::string, int> tuning_params;

    // Read the current value of each trackbar and store it in the map
    for (const auto& name : trackbar_names) {
        tuning_params[name] = cv::getTrackbarPos(name, "Tuning");
    }

    // Return the map of trackbar values
    return tuning_params;
}

// Define a function to wait for a short period of time, allowing GUI events to be processed
void wait_on_gui() {
    cv::waitKey(2);
}