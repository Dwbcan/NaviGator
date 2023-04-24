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
cv::KeyPoint normalise_keypoint(cv::Mat cv_image, cv::KeyPoint kp) {
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