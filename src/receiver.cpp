#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>
#include <math.h>
#include <cassert>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>


#define FOCAL_LENGTH_IN_PIXELS 530.8560
#define REAL_RADIUS 0.3


//y(k+1) = alpha * x(k + 1) + (1 - alpha) * y(k)
// http://nghiaho.com/?p=2093
double alpha_radius = 0.7;
double alpha_center = 0.6;
static const std::string OPENCV_WINDOW = "Image window";
static const std::string GRAY_IMG_WINDOW = "GRAY IMAGE WINDOW";



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher centerPublisher;
  cv::Mat previous_frame;
  double acc_radius;
  double acc_center_x;
  double acc_center_y;
  bool received_image;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    centerPublisher = nh_.advertise<geometry_msgs::PoseStamped>("/image_converter/image_feedback",1);
    acc_radius = 47;
    acc_center_y = 300;
    acc_center_x = 500;
    received_image = false;
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
  
  
  double heightToObject;
  double horizontalDistanceToObject;
  double angleToObject;
  cv::medianBlur(cv_ptr->image,cv_ptr->image,11);
  cv::Mat hsv_image;
  cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

  cv::Mat lower_blue_hue_range;
  cv::Mat upper_blue_hue_range;

  cv::inRange(hsv_image, cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255), lower_blue_hue_range);
	//cv::inRange(hsv_image, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), upper_blue_hue_range);
  
  cv::GaussianBlur(lower_blue_hue_range, lower_blue_hue_range, cv::Size(9, 9), 2, 2);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10));
  cv::erode(lower_blue_hue_range, lower_blue_hue_range,element);
  cv::dilate(lower_blue_hue_range, lower_blue_hue_range,element);

  cv::Mat lower_red_hue_range;
 	cv::Mat upper_red_hue_range;
 	cv::inRange(hsv_image, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), lower_red_hue_range);
	cv::inRange(hsv_image, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), upper_red_hue_range);

  

  cv::Mat red_hue_image;
	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
  cv::addWeighted(red_hue_image, 1.0, lower_blue_hue_range, 1.0, 0.0, red_hue_image);

  cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
  cv::erode(red_hue_image, red_hue_image,element);
  cv::dilate(red_hue_image, red_hue_image,element);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(red_hue_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1);
  cv::drawContours(cv_ptr->image, contours, -1, cv::Scalar(0, 255, 0), 1, 8, hierarchy, 2);

  /*std::vector<cv::Vec3f> circles;
  cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 200, 30, 0, 0);
  geometry_msgs::PoseStamped current_feedback;
  
  
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
 	}*/

    cv::imshow(GRAY_IMG_WINDOW, red_hue_image);
    cv::imshow(OPENCV_WINDOW,cv_ptr->image);
    cv::waitKey(3);
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