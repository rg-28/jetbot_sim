#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "iostream"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

using namespace cv;
using namespace std;
RNG rng;

static const std::string OPENCV_WINDOW = "Image window";

std_msgs::Float64 send_vel;

vector<Point2f> centroid(3);

Mat lane_detect(Mat& lanes);

class ImageConverter
{
  ros::NodeHandle nh; 
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  //Publish the linear and angular velocity to /cmd_vel topic
  ros::Publisher bot_velocity_pub = nh.advertise<std_msgs::Float64>("/error", 1);
  ImageConverter()
    : it_(nh)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera1/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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
    double image_width = cv_ptr->image.size().width;

    // Detect the lanes from the received image
    cv_ptr->image = lane_detect(cv_ptr->image);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

    //Calculate the required set point
    double set_point = image_width / 2;

    //Calculate the error between the set point and current state of the bot
    std_msgs::Float64 error_x;
    error_x.data = double(centroid[2].x - set_point);
    
    //Send the variable angular z velocity to the bot
    send_vel = error_x;
    bot_velocity_pub.publish(send_vel);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

Mat lane_detect(Mat& lanes){
  
  int offset_x = 0;
  int offset_y = 160;

  Rect roi;
  roi.x = offset_x;
  roi.y = offset_y;
  roi.width = lanes.size().width - (offset_x*2);
  roi.height = lanes.size().height - (offset_y*2);

  /* Crop the original image to the defined ROI */

  Mat crop = lanes(roi);

  // Convert BGR image to HSV image.
  Mat bgr2hsv = Mat::zeros(crop.rows,crop.cols, CV_8UC3);
  cvtColor(crop, bgr2hsv, COLOR_BGR2HSV);

  // Apply binary mask on the HSV image.
  Mat binary_img = Mat::zeros(crop.rows,crop.cols, CV_8UC3);
  inRange(bgr2hsv, Scalar(20, 100, 100), Scalar(30, 255, 255), binary_img);

  // Combine the binary masked image with the HSV image.
  Mat merged_img = Mat::zeros(crop.rows,crop.cols, CV_8UC3);
  bitwise_and(bgr2hsv, bgr2hsv, merged_img, binary_img);

  // Convert the merged image to gray scale.
  Mat merged_img_gray = Mat::zeros(crop.rows,crop.cols, CV_8UC1);
  cvtColor(merged_img, merged_img_gray, COLOR_HSV2BGR);
  cvtColor(merged_img_gray, merged_img_gray, COLOR_BGR2GRAY);
  blur(merged_img_gray, merged_img_gray, Size(3,3));
  
  // Find out the contours from the image.
  Mat canny_op;
  int thresh = 100;
  Canny(merged_img_gray, canny_op, thresh, thresh*2);
  vector<vector<Point> > contours;
  findContours( canny_op, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    vector<Moments> mu(contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        mu[i] = moments( contours[i] );
    }
    vector<Point2f> mc( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        //add 1e-5 to avoid division by zero
        mc[i] = Point2f( static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
                         static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)) );
        // cout << "mc[" << i << "]=" << mc[i] << endl;
    }
    
    //Calculate the centroid of the centers of masses of the contours extracted from the image
    centroid[0].x = (mc[0].x + mc[1].x)/2;
    centroid[0].y = (mc[0].y + mc[1].y)/2;
    centroid[1].x = (mc[2].x + mc[3].x)/2;
    centroid[1].y = (mc[3].y + mc[3].y)/2;
    centroid[2].x = (mc[0].x + mc[1].x + mc[2].x + mc[3].x)/4;
    centroid[2].y = (mc[0].y + mc[1].y + mc[2].y + mc[3].y)/4;

    //Draw the center of masses on the output image for reference
    Mat drawing = Mat::zeros( canny_op.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar(0, 0, 255);
        drawContours( drawing, contours, (int)i, color, 2 );
        circle( drawing, centroid[i], 4, color, -1 );
    }

  return drawing;
}