#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <drone_img_proc/LandTarget.h>
#include <numeric>
#include <math.h>
#include <cassert>
#include <fstream>


#define FOCAL_LENGTH_IN_PIXELS 530.8560
#define REAL_RADIUS 0.3

//y(k+1) = alpha * x(k + 1) + (1 - alpha) * y(k)
// http://nghiaho.com/?p=2093
double alpha_radius = 0.7;
double alpha_center = 0.6;
static const std::string OPENCV_WINDOW = "Image window";
static const std::string GRAY_IMG_WINDOW = "GRAY IMAGE WINDOW";

using namespace std;
using namespace cv;

// This video stablisation smooths the global trajectory using a sliding average window

const int SMOOTHING_RADIUS = 30; // In frames. The larger the more stable the video, but less reactive to sudden panning
const int HORIZONTAL_BORDER_CROP = 20; // In pixels. Crops the border to reduce the black borders from stabilisation being too noticeable.

// 1. Get previous to current frame transformation (dx, dy, da) for all frames
// 2. Accumulate the transformations to get the image trajectory
// 3. Smooth out the trajectory using an averaging window
// 4. Generate new set of previous to current transform, such that the trajectory ends up being the same as the smoothed trajectory
// 5. Apply the new transformation to the video

struct TransformParam
{
    TransformParam() {}
    TransformParam(double _dx, double _dy, double _da) {
        dx = _dx;
        dy = _dy;
        da = _da;
    }

    double dx;
    double dy;
    double da; // angle
};

struct Trajectory
{
    Trajectory() {}
    Trajectory(double _x, double _y, double _a) {
        x = _x;
        y = _y;
        a = _a;
    }

    double x;
    double y;
    double a; // angle
};




class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher centerPublisher;
  double acc_radius;
  double acc_center_x;
  double acc_center_y;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    centerPublisher = nh_.advertise<drone_img_proc::LandTarget>("/image_converter/image_feedback",10);
    acc_radius = 47;
    acc_center_y = 300;
    acc_center_x = 500;
    cv::namedWindow(GRAY_IMG_WINDOW);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyAllWindows();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	// Convert to grayscale
	/*cv::Mat gray;
	cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

	// Use Canny instead of threshold to catch squares with gradient shading
	cv::Mat bw;
	//cv::Canny(gray, bw, 100, 300, 5);
  cv::threshold(gray, bw, 70, 255,1);

	// Find contours
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;
	cv::Mat dst = cv_ptr->image.clone();

	for (int i = 0; i < contours.size(); i++)
	{
        int count = 0;
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

		// Skip small or non-convex objects 
		if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
			continue;

		
			// Detect and label circles
			double area = cv::contourArea(contours[i]);
			cv::Rect r = cv::boundingRect(contours[i]);
			int radius = r.width / 2;
      float test_radius;
      cv::Point2f test_center;
			if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
			    std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2){
                    test_center.x = r.x + (r.width/2);
                    test_center.y = r.y + (r.height/2);
                    cv::circle(cv_ptr->image, test_center,radius,cv::Scalar(255,0,0),2);
                    count++;
                    ROS_INFO("CIRCLE COUNT:%d",count);
                }
				
		
	}

	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::imshow(GRAY_IMG_WINDOW, bw);
  cv::waitKey(3);*/


	// Check if the image can be loaded

  Mat cur, cur_grey;
  Mat prev, prev_grey;


  cvtColor(prev, prev_grey, COLOR_BGR2GRAY);

  std::vector <TransformParam> prev_to_cur_transform;

  drone_img_proc::LandTarget nav_target;
  double heightToObject;
  double horizontalDistanceToObject;
  double angleToObject;
  cv::Mat backup_image = cv_ptr->image.clone();
  cv::medianBlur(cv_ptr->image,cv_ptr->image,11);
  cv::Mat hsv_image;
  cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

  cv::Mat lower_red_hue_range;
 	cv::Mat upper_red_hue_range;
 	cv::inRange(hsv_image, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), lower_red_hue_range);
	cv::inRange(hsv_image, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), upper_red_hue_range);

  cv::Mat red_hue_image;
	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

  cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10));
  cv::erode(red_hue_image, red_hue_image,element);
  cv::dilate(red_hue_image, red_hue_image,element);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 200, 30, 0, 0);

  
 	for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
 		cv::Point curr_center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
    acc_center_x = ((alpha_center) * curr_center.x) + ((1 - alpha_center) * acc_center_x);
    acc_center_y = ((alpha_center) * curr_center.y) + ((1 - alpha_center) * acc_center_y);
    cv::Point center(acc_center_x,acc_center_y);
 		int radius = std::round(circles[current_circle][2]);
 		cv::circle(cv_ptr->image, center, radius, cv::Scalar(0, 255, 0), 5);
    acc_radius = ((alpha_radius) * circles[current_circle][2]) + ((1 - alpha_radius) * acc_radius);
    heightToObject = FOCAL_LENGTH_IN_PIXELS * REAL_RADIUS / acc_radius;
    double distanceToCenter;
    cv::Point difference = center - cv::Point(640,360);
    distanceToCenter = std::sqrt((difference.x * difference.x) + (difference.y * difference.y));
    horizontalDistanceToObject = REAL_RADIUS * distanceToCenter / acc_radius;
    angleToObject = atan2(heightToObject,horizontalDistanceToObject);
    //ROS_INFO("MESSAGE DATA:%f %f %f",heightToObject,horizontalDistanceToObject,angleToObject);
    nav_target.stamp = ros::Time::now();
    nav_target.centerFound = true;
    nav_target.distanceToCenter = std::sqrt(horizontalDistanceToObject * horizontalDistanceToObject + heightToObject * heightToObject);
    nav_target.angleToCenter = angleToObject;
    centerPublisher.publish(nav_target);  
 	}

  
  cv::imshow(OPENCV_WINDOW,cv_ptr->image);
  cv::imshow(GRAY_IMG_WINDOW,red_hue_image);
  cv::waitKey(3);

    // Draw an example circle on the video stream
    /*if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));*/
    /*cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(9,9), 2, 2);
    cv::threshold(gray, gray, 70, 255,1);
    std::vector<cv::Vec3f> circles;  
    std::vector<std::vector<cv::Point> > contours; // Vector for storing contour
    std::vector<cv::Vec4i> hierarchy;
    findContours(gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++) {
      float area = contourArea(contours[i], false);
      if (area > 300) {
        drawContours(cv_ptr->image, contours, i, Scalar(0, 0, 255), 2, 8, hierarchy);
      }
  }

  cv::imshow(OPENCV_WINDOW,cv_ptr->image);
  cv::imshow(GRAY_IMG_WINDOW,gray);
  cv::waitKey(3);*/
    /*cv::Mat canny;

    cv::Mat gray;
    /// Convert it to gray
    cv::cvtColor( cv_ptr->image, gray, CV_BGR2GRAY );

    // compute canny (don't blur with that image quality!!)
    cv::GaussianBlur(gray,gray, cv::Size(9,9),2,2);
    cv::Canny(gray, canny, 300,100);
     
    

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    cv::HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 3, 50, 300, 20, 0, 0 );

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ ) 
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle( cv_ptr->image, center, 3, cv::Scalar(0,255,255), -1);
        cv::circle( cv_ptr->image, center, radius, cv::Scalar(0,0,255), 1 );
    }

    //compute distance transform:
    cv::Mat dt;
    cv::distanceTransform(255-(canny>0), dt, CV_DIST_L2 ,3); 
    

    // test for semi-circles:
    float minInlierDist = 2.0f;
    for( size_t i = 0; i < 5; i++ ) 
    {
        // test inlier percentage:
        // sample the circle and check for distance to the next edge
        unsigned int counter = 0;
        unsigned int inlier = 0;

        cv::Point2f center((circles[i][0]), (circles[i][1]));
        float radius = (circles[i][2]);

        // maximal distance of inlier might depend on the size of the circle
        float maxInlierDist = radius/25.0f;
        if(maxInlierDist<minInlierDist) maxInlierDist = minInlierDist;

        //TODO: maybe paramter incrementation might depend on circle size!
        for(float t =0; t<2*3.14159265359f; t+= 0.1f) 
        {
            counter++;
            float cX = radius*cos(t) + circles[i][0];
            float cY = radius*sin(t) + circles[i][1];

            if(dt.at<float>(cY,cX) < maxInlierDist) 
            {
                inlier++;
                cv::circle(cv_ptr->image, cv::Point2i(cX,cY),3, cv::Scalar(0,255,0));
            } 
           else
                cv::circle(cv_ptr->image, cv::Point2i(cX,cY),3, cv::Scalar(255,0,0));
        }
        ROS_INFO("%f  of a circle with radius %f detected",(100.0f*(float)inlier/(float)counter),radius);
    } 
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(GRAY_IMG_WINDOW, dt);
    cv::waitKey(3);*/

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}